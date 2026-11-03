#include <moveit/task_constructor/job_scheduler_p.h>
#include <moveit/task_constructor/task_p.h>

#include "stage_mockups.h"
#include "models.h"

#include <atomic>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace moveit::task_constructor;
using namespace std::chrono_literals;
using moveit::core::MoveItErrorCode;
using testing::Each;
using testing::ElementsAre;
using testing::ElementsAreArray;
using testing::Eq;
using testing::HasSubstr;
using testing::IsEmpty;
using testing::Not;

// plan() runs the job path of the compute stages, but synchronously with an inline JobScheduler(0)
struct InlineJobs : TaskTestBase
{
	InlineJobs() { t.pimpl()->setInlineJobs(true); }
};

TEST_F(InlineJobs, ComputeRunsInsideJob) {
	add(t, new GeneratorMockup({ 0 }));
	auto fwd = add(t, new ForwardMockup());
	bool in_job = false;
	fwd->on_compute = [&in_job] { in_job = Job::current() != nullptr; };

	EXPECT_TRUE(t.plan());
	EXPECT_TRUE(in_job);
	EXPECT_EQ(t.numSolutions(), 1u);
}

TEST_F(InlineJobs, DisabledParallelComputeRunsDirectly) {
	add(t, new GeneratorMockup({ 0 }));
	auto fwd = add(t, new ForwardMockup());
	fwd->setParallelCompute(false);
	bool in_job = true;
	fwd->on_compute = [&in_job] { in_job = Job::current() != nullptr; };

	EXPECT_TRUE(t.plan());
	EXPECT_FALSE(in_job);
	EXPECT_EQ(t.numSolutions(), 1u);
}

// propagator reporting a failure without a failure solution
struct SilentFailureMockup : ForwardMockup
{
	void computeForward(const InterfaceState& /*from*/) override { silentFailure(); }
};

TEST_F(InlineJobs, SilentFailureIsCounted) {
	add(t, new GeneratorMockup({ 0 }));
	auto fwd = add(t, new SilentFailureMockup());

	EXPECT_FALSE(t.plan());
	EXPECT_EQ(fwd->numFailures(), 1u);
}

TEST_F(InlineJobs, PropertyErrorStillReported) {
	add(t, new GeneratorMockup({ 0 }));
	auto fwd = add(t, new ForwardMockup());
	fwd->on_compute = [fwd] { fwd->properties().get<double>("undeclared"); };

	try {
		t.plan();
		ADD_FAILURE() << "expected std::runtime_error";
	} catch (const std::runtime_error& e) {
		EXPECT_THAT(e.what(), HasSubstr("in stage '" + fwd->name() + "'"));
	}
}

// guards the default path (numThreads() == 1): compute() runs directly, not as a job
TEST_F(InlineJobs, WithoutInlineJobsNoJobIsUsed) {
	t.pimpl()->setInlineJobs(false);
	add(t, new GeneratorMockup({ 0 }));
	auto fwd = add(t, new ForwardMockup());
	bool in_job = true;
	fwd->on_compute = [&in_job] { in_job = Job::current() != nullptr; };

	EXPECT_TRUE(t.plan());
	EXPECT_FALSE(in_job);
	EXPECT_EQ(t.numSolutions(), 1u);
}

// plan() computes the stages with 4 worker threads
struct ParallelPlanning : TaskTestBase
{
	ParallelPlanning() { t.setNumThreads(4); }
};

namespace {
// let the compute() of a mockup simulate some work
template <typename S>
S* withWork(S* stage, std::chrono::milliseconds duration, ConcurrencyProbe* probe = nullptr) {
	stage->on_compute = simulateWork(duration, probe);
	return stage;
}

template <typename S>
Stage::pointer slow(S* stage) {
	return Stage::pointer(withWork(stage, 2ms));
}

// prepare a fresh task for planning with num_threads
void setup(Task& task, unsigned int num_threads) {
	task.setRobotModel(getModel());
	task.setNumThreads(num_threads);
}

// GEN{0} → Alternatives{4 × FWD}
void buildAlternatives(Task& task, ConcurrencyProbe& probe) {
	task.add(Stage::pointer(new GeneratorMockup({ 0 })));
	auto alternatives = std::make_unique<Alternatives>();
	for (int i = 0; i < 4; ++i)
		alternatives->add(Stage::pointer(withWork(new ForwardMockup(), 50ms, &probe)));
	task.add(std::move(alternatives));
}

// GEN{0,0,0} → FWD c1 → FWD c2 ⇒ {3,3,3}
void buildChain(Task& task) {
	task.add(slow(new GeneratorMockup({ 0, 0, 0 })));
	task.add(slow(new ForwardMockup(PredefinedCosts::constant(1.0))));
	task.add(slow(new ForwardMockup(PredefinedCosts::constant(2.0))));
}

// GEN{0,0} → Alternatives{FWD c1, FWD c2, FWD c3} ⇒ {1,1,2,2,3,3}
void buildAlternativesChain(Task& task) {
	task.add(slow(new GeneratorMockup({ 0, 0 })));
	auto alternatives = std::make_unique<Alternatives>();
	for (double cost : { 1.0, 2.0, 3.0 })
		alternatives->add(slow(new ForwardMockup(PredefinedCosts::constant(cost))));
	task.add(std::move(alternatives));
}

// GEN{0,0,0} → CON c1 → GEN{0,0} ⇒ six × 1
void buildConnect(Task& task) {
	task.add(slow(new GeneratorMockup({ 0, 0, 0 })));
	task.add(slow(new ConnectMockup(PredefinedCosts::constant(1.0))));
	task.add(slow(new GeneratorMockup({ 0, 0 })));
}

// GEN{0,0} → Serial{FWD c1, Alternatives{FWD c1, FWD c2}} ⇒ {2,2,3,3}
void buildNested(Task& task) {
	task.add(slow(new GeneratorMockup({ 0, 0 })));
	auto serial = std::make_unique<SerialContainer>();
	serial->add(slow(new ForwardMockup(PredefinedCosts::constant(1.0))));
	auto alternatives = std::make_unique<Alternatives>();
	alternatives->add(slow(new ForwardMockup(PredefinedCosts::constant(1.0))));
	alternatives->add(slow(new ForwardMockup(PredefinedCosts::constant(2.0))));
	serial->add(std::move(alternatives));
	task.add(std::move(serial));
}

// GEN produces states forever, but FWD fails on each of them
void buildFailingChain(Task& task) {
	task.add(std::make_unique<GeneratorMockup>(PredefinedCosts::constant(0.0)));
	task.add(Stage::pointer(withWork(new ForwardMockup(PredefinedCosts::constant(INF)), 20ms)));
}
}  // namespace

TEST_F(ParallelPlanning, RunsComputeConcurrently) {
	ConcurrencyProbe probe;
	buildAlternatives(t, probe);
	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.numSolutions(), 4u);
	EXPECT_GE(probe.max.load(), 2);

	ConcurrencyProbe sequential_probe;
	Task sequential;
	setup(sequential, 1);
	buildAlternatives(sequential, sequential_probe);
	EXPECT_TRUE(sequential.plan());
	EXPECT_EQ(sequential.numSolutions(), 4u);
	EXPECT_EQ(sequential_probe.max.load(), 1);
}

TEST_F(ParallelPlanning, MatchesSequentialResults) {
	struct Topology
	{
		const char* name;
		std::function<void(Task&)> build;
		std::vector<double> expected_costs;
	};
	const std::vector<Topology> topologies{
		{ "chain", buildChain, { 3, 3, 3 } },
		{ "alternatives", buildAlternativesChain, { 1, 1, 2, 2, 3, 3 } },
		{ "connect", buildConnect, { 1, 1, 1, 1, 1, 1 } },
		{ "nested", buildNested, { 2, 2, 3, 3 } },
	};
	for (const Topology& topology : topologies) {
		for (int run = 0; run < 20; ++run) {
			SCOPED_TRACE(std::string(topology.name) + ", run " + std::to_string(run));
			Task task;
			setup(task, 4);
			topology.build(task);
			EXPECT_TRUE(task.plan());
			EXPECT_COSTS(task.solutions(), ElementsAreArray(topology.expected_costs));
		}
	}
}

TEST_F(ParallelPlanning, ComputeOnWorkerCallbacksOnPlanningThread) {
	std::vector<std::thread::id> compute_ids, task_cb_ids, solution_cb_ids;
	add(t, new GeneratorMockup({ 0 }));
	auto fwd = add(t, new ForwardMockup());
	fwd->on_compute = [&compute_ids] { compute_ids.push_back(std::this_thread::get_id()); };
	t.addTaskCallback([&task_cb_ids](const Task& /*task*/) { task_cb_ids.push_back(std::this_thread::get_id()); });
	t.addSolutionCallback(
	    [&solution_cb_ids](const SolutionBase& /*s*/) { solution_cb_ids.push_back(std::this_thread::get_id()); });

	EXPECT_TRUE(t.plan());
	const std::thread::id planning_thread = std::this_thread::get_id();
	ASSERT_EQ(compute_ids.size(), 1u);
	EXPECT_NE(compute_ids.front(), planning_thread);
	EXPECT_THAT(task_cb_ids, Not(IsEmpty()));
	EXPECT_THAT(task_cb_ids, Each(Eq(planning_thread)));
	EXPECT_THAT(solution_cb_ids, ElementsAre(planning_thread));
}

TEST_F(ParallelPlanning, Preempt) {
	buildFailingChain(t);
	t.setTimeout(5.0);  // safety net: a broken preempt must not let the test hang

	MoveItErrorCode result;
	std::chrono::steady_clock::time_point returned;
	std::thread planning([&] {
		try {
			result = t.plan();
		} catch (const std::exception& e) {
			ADD_FAILURE() << "plan() threw: " << e.what();
		}
		returned = std::chrono::steady_clock::now();
	});
	std::this_thread::sleep_for(100ms);
	const auto preempted = std::chrono::steady_clock::now();
	t.preempt();
	planning.join();

	EXPECT_EQ(result.val, MoveItErrorCode::PREEMPTED);
	EXPECT_LT(returned - preempted, 500ms);
	EXPECT_FALSE(t.stages()->pimpl()->busy());

	// a stale busy flag would end this run early with PLANNING_FAILED
	t.setTimeout(0.3);
	EXPECT_EQ(t.plan().val, MoveItErrorCode::TIMED_OUT);
}

TEST_F(ParallelPlanning, Timeout) {
	buildFailingChain(t);
	t.setTimeout(0.2);

	const auto start = std::chrono::steady_clock::now();
	EXPECT_EQ(t.plan().val, MoveItErrorCode::TIMED_OUT);
	EXPECT_LT(std::chrono::steady_clock::now() - start, 500ms);
}

TEST_F(ParallelPlanning, MaxSolutions) {
	add(t, new GeneratorMockup(PredefinedCosts::constant(0.0)));
	add(t, withWork(new ForwardMockup(PredefinedCosts::constant(1.0)), 5ms));
	t.setTimeout(5.0);  // safety net: ignoring max_solutions must not let the test hang

	EXPECT_EQ(t.plan(3).val, MoveItErrorCode::SUCCESS);
	EXPECT_GE(t.numSolutions(), 3u);
	EXPECT_LE(t.numSolutions(), 7u);
}

TEST_F(ParallelPlanning, WorkerExceptionIsRethrownAfterOthersFinish) {
	std::atomic<bool> started{ false }, finished{ false };
	add(t, new GeneratorMockup({ 0 }));
	auto alternatives = add(t, new Alternatives());
	auto failing = add(*alternatives, new ForwardMockup());
	failing->on_compute = [&started] {
		const auto deadline = std::chrono::steady_clock::now() + 1s;
		while (!started && std::chrono::steady_clock::now() < deadline)
			std::this_thread::sleep_for(1ms);
		throw std::runtime_error("boom");
	};
	auto working = add(*alternatives, new ForwardMockup());
	working->on_compute = [&started, &finished] {
		started = true;
		std::this_thread::sleep_for(100ms);
		finished = true;
	};

	EXPECT_THROW(t.plan(), std::runtime_error);
	EXPECT_TRUE(finished);
	EXPECT_EQ(t.numSolutions(), 1u);  // the result of the job, which was still running, got committed
	EXPECT_FALSE(t.stages()->pimpl()->busy());
}

TEST_F(ParallelPlanning, MonitoringGeneratorReceivesSolutionsWhileBusy) {
	for (int run = 0; run < 10; ++run) {
		SCOPED_TRACE("run " + std::to_string(run));
		Task task;
		setup(task, 4);
		auto gen = withWork(new GeneratorMockup({ 0, 0, 0, 0, 0 }), 5ms);
		task.add(Stage::pointer(gen));
		task.add(std::make_unique<ConnectMockup>(PredefinedCosts::constant(0.0)));
		auto qmon = withWork(new QueueingMonitoringGeneratorMockup(gen), 20ms);
		task.add(Stage::pointer(qmon));

		EXPECT_TRUE(task.plan());
		EXPECT_EQ(task.numSolutions(), 25u);
		EXPECT_EQ(qmon->runs_, 5u);
	}
}

TEST_F(ParallelPlanning, ComputeTimeIsAccumulated) {
	add(t, new GeneratorMockup({ 0, 0, 0 }));
	auto fwd = add(t, withWork(new ForwardMockup(), 20ms));

	EXPECT_TRUE(t.plan());
	EXPECT_GE(fwd->getTotalComputeTime(), 0.06);
}

TEST_F(ParallelPlanning, NumThreadsSurvivesMove) {
	Task a;
	EXPECT_EQ(a.numThreads(), 1u);  // default: sequential planning
	a.setNumThreads(3);
	Task b(std::move(a));
	EXPECT_EQ(b.numThreads(), 3u);
}

TEST_F(ParallelPlanning, ZeroMeansHardwareConcurrency) {
	t.setNumThreads(0);
	EXPECT_EQ(t.numThreads(), 0u);

	buildChain(t);
	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ElementsAre(3, 3, 3));
}
