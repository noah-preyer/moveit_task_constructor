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

/* Desc: Generic thread pool running jobs with exclusive resources (private API) */

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

namespace moveit {
namespace task_constructor {

/** A unit of work for the JobScheduler.
 *
 *  `work` runs on a worker thread (in inline mode: on the submitting thread). `complete` and `cancel`
 *  run on the planning thread, i.e. the thread calling JobScheduler::commitFinished(), drain() or
 *  abort() (in inline mode: submit()). Actions that must happen on the planning thread are recorded
 *  during `work` via defer() and replayed from `complete`.
 */
struct Job
{
	const void* owner = nullptr;  ///< opaque id of the submitter (the StagePrivate*)
	std::function<void()> work;  ///< runs on a worker (inline mode: calling thread)
	std::vector<const void*> resources;  ///< held exclusively while `work` runs
	std::function<void(Job&)> complete;  ///< planning thread, after `work` finished
	std::function<void(Job&)> cancel;  ///< planning thread, for jobs discarded by drain()/abort()
	std::vector<std::function<void()>> commits;  ///< filled through defer() during `work`
	std::exception_ptr error;  ///< set if `work` threw
	std::chrono::duration<double> duration{};  ///< wall time of `work`

	/// Record an action for `complete` to replay on the planning thread (call from `work`)
	void defer(std::function<void()>&& commit) { commits.push_back(std::move(commit)); }

	/// The job whose `work` currently runs on the calling thread, otherwise nullptr
	static Job* current();
};

/** Thread pool that runs Jobs and hands the finished ones back to the planning thread.
 *
 *  The scheduler knows nothing about stages. Workers pick the first queued job (FIFO) whose
 *  `resources` are not held by any running job, run its `work` and put it into the list of finished
 *  jobs. Finished jobs are processed by the planning thread in finish order via commitFinished().
 *
 *  Exceptions thrown by `work` are caught on the worker and stored in Job::error. Exceptions thrown by
 *  callbacks propagate out of the function that invokes them (swallowed by abort()).
 *
 *  Apart from Job::current(), all members must be called from the single planning thread only.
 */
class JobScheduler
{
public:
	/// Start `num_threads` workers. With 0 threads, jobs run inline in submit() (no threads, no resources).
	explicit JobScheduler(unsigned int num_threads);
	/// abort(), then stop and join all workers
	~JobScheduler();

	JobScheduler(const JobScheduler&) = delete;
	JobScheduler& operator=(const JobScheduler&) = delete;

	/** Queue a job for execution.
	 *  Inline mode: run `work`, then `complete`, then rethrow Job::error (if still set), all on the caller's thread.
	 */
	void submit(Job&& job);

	/// Number of submitted jobs that are queued, running, or finished but not yet completed/cancelled
	std::size_t inflight() const;

	/** Wait until a finished job is available or the deadline passed.
	 *  @return true if at least one finished job is waiting to be committed. Returns false right away
	 *          if no job is queued or running, i.e. if nothing could ever finish.
	 */
	bool waitForFinished(std::chrono::steady_clock::time_point deadline);

	/** Run `complete` of all finished jobs in finish order. Job errors are not thrown, but recorded in error().
	 *  @return number of jobs processed
	 */
	std::size_t commitFinished();

	/// First Job::error of a committed job, read *after* its `complete` ran (null if there was none)
	std::exception_ptr error() const;

	/// Cancel queued jobs, wait for running jobs, and commitFinished() them
	void drain();

	/** Cancel queued jobs, wait for running jobs, and cancel those and all finished-but-uncommitted ones as well.
	 *  Neither `work`'s results nor `complete` are used. Idempotent; exceptions thrown by `cancel` are swallowed.
	 */
	void abort() noexcept;

private:
	using JobPtr = std::unique_ptr<Job>;

	void workerLoop();
	/// stop and join all workers
	void shutdownWorkers() noexcept;
	/// first queued job whose resources are all free, or queued_.end(); requires the mutex to be held
	std::list<JobPtr>::iterator firstRunnable();

	mutable std::mutex mutex_;  // guards all members below
	std::condition_variable cv_work_;  // wakes workers: new job, freed resources, shutdown
	std::condition_variable cv_finished_;  // wakes the planning thread: job finished
	std::list<JobPtr> queued_;  // waiting for a worker
	std::size_t running_ = 0;  // jobs whose `work` currently runs
	std::deque<JobPtr> finished_;  // work done, waiting for commitFinished()
	std::set<const void*> held_resources_;  // resources of running jobs
	std::exception_ptr error_;  // first error of a committed job
	bool stop_ = false;  // tells the workers to terminate

	std::vector<std::thread> workers_;  // only touched by the constructor and the destructor
};

}  // namespace task_constructor
}  // namespace moveit
