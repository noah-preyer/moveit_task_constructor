/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, MoveIt Task Constructor contributors
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Desc: Generic thread pool running jobs with exclusive resources */

#include <moveit/task_constructor/job_scheduler_p.h>

#include <algorithm>
#include <utility>

namespace moveit {
namespace task_constructor {

namespace {

thread_local Job* current_job = nullptr;

// Run job.work, recording the duration and any exception in the job. Never throws.
void runWork(Job& job) noexcept {
	Job* previous = current_job;
	current_job = &job;
	const auto start = std::chrono::steady_clock::now();
	try {
		if (job.work)
			job.work();
	} catch (...) {
		job.error = std::current_exception();
	}
	job.duration = std::chrono::steady_clock::now() - start;
	current_job = previous;
}

}  // namespace

Job* Job::current() {
	return current_job;
}

JobScheduler::JobScheduler(unsigned int num_threads) {
	try {
		workers_.reserve(num_threads);
		for (unsigned int i = 0; i < num_threads; ++i)
			workers_.emplace_back([this] { workerLoop(); });
	} catch (...) {
		shutdownWorkers();  // a joinable std::thread must not be destroyed
		throw;
	}
}

JobScheduler::~JobScheduler() {
	abort();
	shutdownWorkers();
}

void JobScheduler::shutdownWorkers() noexcept {
	{
		std::lock_guard<std::mutex> lock(mutex_);
		stop_ = true;
	}
	cv_work_.notify_all();
	for (std::thread& worker : workers_)
		worker.join();
	workers_.clear();
}

std::list<JobScheduler::JobPtr>::iterator JobScheduler::firstRunnable() {
	return std::find_if(queued_.begin(), queued_.end(), [this](const JobPtr& job) {
		return std::none_of(job->resources.begin(), job->resources.end(),
		                    [this](const void* resource) { return held_resources_.count(resource) > 0; });
	});
}

void JobScheduler::workerLoop() {
	std::unique_lock<std::mutex> lock(mutex_);
	auto next = queued_.end();
	while (true) {
		cv_work_.wait(lock, [&] {
			next = firstRunnable();
			return stop_ || next != queued_.end();
		});
		if (stop_)
			return;  // abort() has emptied the queue before stop_ is set

		JobPtr job = std::move(*next);
		queued_.erase(next);
		held_resources_.insert(job->resources.begin(), job->resources.end());
		++running_;

		lock.unlock();
		runWork(*job);
		lock.lock();

		for (const void* resource : job->resources)
			held_resources_.erase(resource);
		--running_;
		const bool released = !job->resources.empty();
		finished_.push_back(std::move(job));

		cv_finished_.notify_all();
		if (released)
			cv_work_.notify_all();  // jobs blocked on our resources might be runnable now
	}
}

void JobScheduler::submit(Job&& job) {
	if (workers_.empty()) {  // inline mode: resources are irrelevant, nothing runs concurrently
		runWork(job);
		if (job.complete)
			job.complete(job);
		if (job.error)
			std::rethrow_exception(job.error);
		return;
	}

	JobPtr queued = std::make_unique<Job>(std::move(job));
	{
		std::lock_guard<std::mutex> lock(mutex_);
		queued_.push_back(std::move(queued));
	}
	cv_work_.notify_one();
}

std::size_t JobScheduler::inflight() const {
	std::lock_guard<std::mutex> lock(mutex_);
	return queued_.size() + running_ + finished_.size();
}

bool JobScheduler::waitForFinished(std::chrono::steady_clock::time_point deadline) {
	std::unique_lock<std::mutex> lock(mutex_);
	cv_finished_.wait_until(lock, deadline, [this] { return !finished_.empty() || (queued_.empty() && running_ == 0); });
	return !finished_.empty();
}

std::size_t JobScheduler::commitFinished() {
	std::size_t count = 0;
	while (true) {
		JobPtr job;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (finished_.empty())
				return count;
			job = std::move(finished_.front());
			finished_.pop_front();
		}

		// If complete() throws, this job is gone, but the remaining finished ones are kept for the next call.
		if (job->complete)
			job->complete(*job);
		++count;

		if (job->error) {
			std::lock_guard<std::mutex> lock(mutex_);
			if (!error_)
				error_ = job->error;
		}
	}
}

std::exception_ptr JobScheduler::error() const {
	std::lock_guard<std::mutex> lock(mutex_);
	return error_;
}

void JobScheduler::drain() {
	std::list<JobPtr> cancelled;
	{
		std::unique_lock<std::mutex> lock(mutex_);
		cancelled.swap(queued_);  // workers cannot start these anymore
		cv_finished_.wait(lock, [this] { return running_ == 0; });
	}

	// Every job needs its cancel/complete callback: a failing callback must not make us skip the remaining ones.
	std::exception_ptr first_error;
	for (JobPtr& job : cancelled) {
		try {
			if (job->cancel)
				job->cancel(*job);
		} catch (...) {
			if (!first_error)
				first_error = std::current_exception();
		}
	}
	commitFinished();  // an exception thrown by complete() propagates, the remaining finished jobs are kept
	if (first_error)
		std::rethrow_exception(first_error);
}

void JobScheduler::abort() noexcept {
	std::list<JobPtr> queued;
	std::deque<JobPtr> finished;
	{
		std::unique_lock<std::mutex> lock(mutex_);
		queued.swap(queued_);  // workers cannot start these anymore
		cv_finished_.wait(lock, [this] { return running_ == 0; });
		finished.swap(finished_);  // includes the jobs that were running above
	}

	// cancel callbacks run on the calling thread without holding the mutex
	const auto cancel = [](JobPtr& job) {
		try {
			if (job->cancel)
				job->cancel(*job);
		} catch (...) {
		}
	};
	std::for_each(finished.begin(), finished.end(), cancel);
	std::for_each(queued.begin(), queued.end(), cancel);
}

}  // namespace task_constructor
}  // namespace moveit
