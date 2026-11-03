#include <moveit/task_constructor/job_scheduler_p.h>

#include "stage_mockups.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace std::chrono_literals;
using namespace moveit::task_constructor;
using testing::ElementsAre;

namespace {

using Clock = std::chrono::steady_clock;

// Upper bound for all waits in this file: a bug should fail a test rather than hang it.
constexpr auto TIMEOUT = 10s;

// Poll for a condition until it holds or the timeout expired.
bool waitUntil(const std::function<bool()>& condition) {
	const auto deadline = Clock::now() + TIMEOUT;
	while (!condition()) {
		if (Clock::now() > deadline)
			return false;
		std::this_thread::sleep_for(1ms);
	}
	return true;
}

// Wait (in slices of 1s) until at least one finished job is available for committing.
bool waitFinished(JobScheduler& s) {
	const auto deadline = Clock::now() + TIMEOUT;
	while (Clock::now() < deadline) {
		if (s.waitForFinished(Clock::now() + 1s))
			return true;
	}
	return false;
}

// Commit finished jobs until `count` of them were completed (or the timeout expired).
std::size_t commitUntil(JobScheduler& s, std::size_t count) {
	const auto deadline = Clock::now() + TIMEOUT;
	std::size_t committed = 0;
	while (committed < count && Clock::now() < deadline) {
		s.waitForFinished(Clock::now() + 1s);
		committed += s.commitFinished();
	}
	return committed;
}

Job sleepingJob(std::chrono::milliseconds duration, std::vector<const void*> resources = {}) {
	Job job;
	job.work = [duration] { std::this_thread::sleep_for(duration); };
	job.resources = std::move(resources);
	return job;
}

}  // namespace

TEST(JobScheduler, InlineRunsWorkAndCompleteSynchronously) {
	JobScheduler s(0);
	bool ran_inside_job = false;
	bool completed = false;

	Job job;
	job.work = [&] { ran_inside_job = Job::current() != nullptr; };
	job.complete = [&](Job&) { completed = true; };
	s.submit(std::move(job));

	EXPECT_TRUE(ran_inside_job);
	EXPECT_TRUE(completed);
	EXPECT_EQ(s.inflight(), 0u);
}

TEST(JobScheduler, InlineRethrowsAfterComplete) {
	JobScheduler s(0);
	bool completed = false;

	Job job;
	job.work = [] { throw std::runtime_error("failed"); };
	job.complete = [&](Job&) { completed = true; };

	EXPECT_THROW(s.submit(std::move(job)), std::runtime_error);
	EXPECT_TRUE(completed);
	EXPECT_EQ(s.inflight(), 0u);
}

TEST(JobScheduler, CurrentIsNullOutsideWork) {
	EXPECT_EQ(Job::current(), nullptr);
}

TEST(JobScheduler, CurrentIsClearedAfterThrowingWork) {
	JobScheduler s(0);
	Job job;
	job.work = [] { throw std::runtime_error("failed"); };
	EXPECT_THROW(s.submit(std::move(job)), std::runtime_error);
	EXPECT_EQ(Job::current(), nullptr);
}

TEST(JobScheduler, CurrentIsOnlySetOnWorkerWhileWorkRuns) {
	JobScheduler s(1);
	std::atomic<const Job*> seen_in_work{ nullptr };
	const Job* completed_job = nullptr;

	Job job;
	job.work = [&] { seen_in_work = Job::current(); };
	job.complete = [&](Job& j) { completed_job = &j; };
	s.submit(std::move(job));

	ASSERT_TRUE(waitFinished(s));
	EXPECT_EQ(s.commitFinished(), 1u);
	EXPECT_NE(seen_in_work.load(), nullptr);
	EXPECT_EQ(seen_in_work.load(), completed_job);  // work and complete see the very same Job object
	EXPECT_EQ(Job::current(), nullptr);  // never set on the planning thread
}

TEST(JobScheduler, DeferredCommitsReplayedOnCallingThread) {
	JobScheduler s(2);
	std::vector<int> values;
	std::thread::id work_thread;
	std::thread::id complete_thread;

	Job job;
	job.work = [&] {
		work_thread = std::this_thread::get_id();
		Job::current()->defer([&values] { values.push_back(1); });
		Job::current()->defer([&values] { values.push_back(2); });
	};
	job.complete = [&](Job& j) {
		complete_thread = std::this_thread::get_id();
		for (auto& c : j.commits)
			c();
	};
	s.submit(std::move(job));

	ASSERT_TRUE(waitFinished(s));
	EXPECT_EQ(s.commitFinished(), 1u);
	EXPECT_THAT(values, ElementsAre(1, 2));
	EXPECT_EQ(complete_thread, std::this_thread::get_id());
	EXPECT_NE(work_thread, std::this_thread::get_id());
}

TEST(JobScheduler, DurationMeasuresWork) {
	JobScheduler s(1);
	double duration = 0.0;

	Job job = sleepingJob(30ms);
	job.complete = [&](Job& j) { duration = j.duration.count(); };
	s.submit(std::move(job));

	EXPECT_EQ(commitUntil(s, 1), 1u);
	EXPECT_GE(duration, 0.03);
}

TEST(JobScheduler, RunsJobsConcurrently) {
	JobScheduler s(4);
	ConcurrencyProbe probe;
	std::atomic<int> completed{ 0 };

	for (int i = 0; i < 4; ++i) {
		Job job;
		job.work = [&probe] {
			ConcurrencyProbe::Scope scope(probe);
			std::this_thread::sleep_for(50ms);
		};
		job.complete = [&](Job&) { ++completed; };
		s.submit(std::move(job));
	}

	EXPECT_EQ(commitUntil(s, 4), 4u);
	EXPECT_EQ(completed, 4);
	EXPECT_GE(probe.max, 2);
	EXPECT_EQ(probe.current, 0);
}

TEST(JobScheduler, SharedResourceSerializesJobs) {
	JobScheduler s(4);
	ConcurrencyProbe probe;
	std::atomic<int> completed{ 0 };
	int r;  // only the address matters

	for (int i = 0; i < 4; ++i) {
		Job job;
		job.work = [&probe] {
			ConcurrencyProbe::Scope scope(probe);
			std::this_thread::sleep_for(50ms);
		};
		job.resources = { &r };
		job.complete = [&](Job&) { ++completed; };
		s.submit(std::move(job));
	}

	EXPECT_EQ(commitUntil(s, 4), 4u);
	EXPECT_EQ(completed, 4);
	EXPECT_EQ(probe.max, 1);
}

TEST(JobScheduler, BlockedJobDoesNotBlockOthers) {
	JobScheduler s(2);
	std::vector<char> order;
	int r;  // only the address matters

	auto submit = [&](char name, std::chrono::milliseconds duration, std::vector<const void*> resources) {
		Job job = sleepingJob(duration, std::move(resources));
		job.complete = [&order, name](Job&) { order.push_back(name); };
		s.submit(std::move(job));
	};
	submit('A', 100ms, { &r });
	submit('B', 10ms, { &r });  // has to wait for A
	submit('C', 10ms, {});  // may overtake the blocked B

	EXPECT_EQ(commitUntil(s, 3), 3u);
	EXPECT_THAT(order, ElementsAre('C', 'A', 'B'));
}

TEST(JobScheduler, ErrorIsRecordedNotThrown) {
	JobScheduler s(2);
	bool completed = false;
	bool error_visible_in_complete = false;

	Job job;
	job.work = [] { throw std::runtime_error("failed"); };
	job.complete = [&](Job& j) {
		completed = true;
		error_visible_in_complete = static_cast<bool>(j.error);
	};
	s.submit(std::move(job));

	ASSERT_TRUE(waitFinished(s));
	EXPECT_FALSE(s.error());
	EXPECT_NO_THROW(s.commitFinished());
	EXPECT_TRUE(s.error());
	EXPECT_TRUE(completed);
	EXPECT_TRUE(error_visible_in_complete);
	EXPECT_THROW(std::rethrow_exception(s.error()), std::runtime_error);
	EXPECT_EQ(s.inflight(), 0u);
}

TEST(JobScheduler, FirstErrorIsKept) {
	JobScheduler s(1);  // single worker: jobs finish in submission order

	for (const char* what : { "first", "second" }) {
		Job job;
		job.work = [what] { throw std::runtime_error(what); };
		s.submit(std::move(job));
	}

	EXPECT_EQ(commitUntil(s, 2), 2u);
	ASSERT_TRUE(s.error());
	try {
		std::rethrow_exception(s.error());
	} catch (const std::runtime_error& e) {
		EXPECT_STREQ(e.what(), "first");
	}
}

TEST(JobScheduler, InflightCountsQueuedRunningAndFinishedJobs) {
	JobScheduler s(1);
	std::atomic<bool> release{ false };
	std::atomic<bool> second_started{ false };
	EXPECT_EQ(s.inflight(), 0u);

	Job first;
	first.work = [] {};
	s.submit(std::move(first));

	Job second;
	second.work = [&] {
		second_started = true;
		waitUntil([&] { return release.load(); });
	};
	s.submit(std::move(second));

	Job third;
	third.work = [] {};
	s.submit(std::move(third));
	EXPECT_EQ(s.inflight(), 3u);

	ASSERT_TRUE(waitUntil([&] { return second_started.load(); }));
	ASSERT_TRUE(waitFinished(s));
	EXPECT_EQ(s.inflight(), 3u);  // finished, but not yet committed; running; queued
	EXPECT_EQ(s.commitFinished(), 1u);
	EXPECT_EQ(s.inflight(), 2u);

	release = true;
	EXPECT_EQ(commitUntil(s, 2), 2u);
	EXPECT_EQ(s.inflight(), 0u);
}

TEST(JobScheduler, DrainCancelsQueuedAndCommitsRunning) {
	JobScheduler s(1);
	std::atomic<bool> started{ false };
	std::vector<char> completed;
	std::vector<char> cancelled;
	bool callbacks_on_calling_thread = true;

	auto submit = [&](char name, std::function<void()> work) {
		Job job;
		job.work = std::move(work);
		const auto caller = std::this_thread::get_id();
		job.complete = [&, name, caller](Job&) {
			callbacks_on_calling_thread &= std::this_thread::get_id() == caller;
			completed.push_back(name);
		};
		job.cancel = [&, name, caller](Job&) {
			callbacks_on_calling_thread &= std::this_thread::get_id() == caller;
			cancelled.push_back(name);
		};
		s.submit(std::move(job));
	};
	submit('A', [&] {
		started = true;
		std::this_thread::sleep_for(50ms);
	});
	submit('B', [] {});
	submit('C', [] {});

	ASSERT_TRUE(waitUntil([&] { return started.load(); }));
	s.drain();

	EXPECT_THAT(completed, ElementsAre('A'));
	EXPECT_THAT(cancelled, ElementsAre('B', 'C'));
	EXPECT_TRUE(callbacks_on_calling_thread);
	EXPECT_EQ(s.inflight(), 0u);
}

TEST(JobScheduler, DrainCommitsFinishedJobs) {
	JobScheduler s(2);
	std::atomic<int> done{ 0 };
	int completed = 0;
	for (int i = 0; i < 2; ++i) {
		Job job;
		job.work = [&done] { ++done; };
		job.complete = [&](Job&) { ++completed; };
		s.submit(std::move(job));
	}
	ASSERT_TRUE(waitUntil([&] { return done == 2; }));

	s.drain();  // commits without a prior commitFinished()
	EXPECT_EQ(completed, 2);
	EXPECT_EQ(s.inflight(), 0u);
}

TEST(JobScheduler, AbortDiscardsRunning) {
	JobScheduler s(1);
	std::atomic<bool> started{ false };
	bool cancelled = false;
	bool completed = false;
	const auto caller = std::this_thread::get_id();
	std::thread::id cancel_thread;

	Job job;
	job.work = [&] {
		started = true;
		std::this_thread::sleep_for(50ms);
	};
	job.complete = [&](Job&) { completed = true; };
	job.cancel = [&](Job&) {
		cancelled = true;
		cancel_thread = std::this_thread::get_id();
	};
	s.submit(std::move(job));

	ASSERT_TRUE(waitUntil([&] { return started.load(); }));
	s.abort();

	EXPECT_TRUE(cancelled);
	EXPECT_FALSE(completed);
	EXPECT_EQ(cancel_thread, caller);
	EXPECT_EQ(s.inflight(), 0u);

	// abort() is idempotent: nothing is cancelled twice
	cancelled = false;
	s.abort();
	EXPECT_FALSE(cancelled);
	EXPECT_EQ(s.inflight(), 0u);
}

TEST(JobScheduler, AbortCancelsFinishedAndQueuedJobs) {
	JobScheduler s(1);
	std::atomic<bool> started{ false };
	std::vector<char> cancelled;
	std::vector<char> completed;

	auto submit = [&](char name, std::function<void()> work) {
		Job job;
		job.work = std::move(work);
		job.complete = [&completed, name](Job&) { completed.push_back(name); };
		job.cancel = [&cancelled, name](Job&) { cancelled.push_back(name); };
		s.submit(std::move(job));
	};
	submit('A', [] {});  // finished, but not committed
	ASSERT_TRUE(waitFinished(s));
	submit('B', [&] {
		started = true;
		std::this_thread::sleep_for(50ms);
	});  // running
	submit('C', [] {});  // queued
	ASSERT_TRUE(waitUntil([&] { return started.load(); }));

	s.abort();
	std::sort(cancelled.begin(), cancelled.end());  // order is not of interest here
	EXPECT_THAT(cancelled, ElementsAre('A', 'B', 'C'));
	EXPECT_TRUE(completed.empty());
	EXPECT_EQ(s.inflight(), 0u);
	EXPECT_EQ(s.commitFinished(), 0u);
}

TEST(JobScheduler, SchedulerIsUsableAfterAbort) {
	JobScheduler s(1);
	s.submit(sleepingJob(10ms));
	s.abort();

	bool completed = false;
	Job job;
	job.work = [] {};
	job.complete = [&](Job&) { completed = true; };
	s.submit(std::move(job));
	EXPECT_EQ(commitUntil(s, 1), 1u);
	EXPECT_TRUE(completed);
}

TEST(JobScheduler, DestructorCancelsPendingJobs) {
	std::atomic<bool> started{ false };
	std::vector<char> cancelled;
	std::vector<char> completed;
	{
		JobScheduler s(1);
		auto submit = [&](char name, std::function<void()> work) {
			Job job;
			job.work = std::move(work);
			job.complete = [&completed, name](Job&) { completed.push_back(name); };
			job.cancel = [&cancelled, name](Job&) { cancelled.push_back(name); };
			s.submit(std::move(job));
		};
		submit('A', [&] {
			started = true;
			std::this_thread::sleep_for(30ms);
		});
		submit('B', [] {});
		ASSERT_TRUE(waitUntil([&] { return started.load(); }));
	}
	std::sort(cancelled.begin(), cancelled.end());  // order is not of interest here
	EXPECT_THAT(cancelled, ElementsAre('A', 'B'));
	EXPECT_TRUE(completed.empty());
}

TEST(JobScheduler, WaitForFinishedHonorsDeadline) {
	JobScheduler s(1);
	s.submit(sleepingJob(200ms));

	const auto start = Clock::now();
	EXPECT_FALSE(s.waitForFinished(start + 20ms));
	EXPECT_LT(Clock::now() - start, 150ms);
}

TEST(JobScheduler, WaitForFinishedDoesNotWaitWithoutJobs) {
	for (unsigned int threads : { 0u, 2u }) {
		JobScheduler s(threads);
		const auto start = Clock::now();
		EXPECT_FALSE(s.waitForFinished(start + TIMEOUT));  // nothing could ever finish
		EXPECT_LT(Clock::now() - start, 1s);
		EXPECT_EQ(s.commitFinished(), 0u);
	}
}
