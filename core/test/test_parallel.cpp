#include <moveit/task_constructor/job_scheduler_p.h>
#include <moveit/task_constructor/task_p.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/multi_planner.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>

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
using testing::UnorderedElementsAre;

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

// the children of Fallbacks compute as jobs, whose results are only committed once they have finished
struct ParallelFallbacks : TaskTestBase
{
	ParallelFallbacks() { t.setNumThreads(4); }
};

// the second child must not be started while the first one is still computing (and might succeed)
TEST_F(ParallelFallbacks, PropagatorDoesNotSkipBusyChild) {
	add(t, new GeneratorMockup({ 0, 0, 0 }));
	auto fallbacks = add(t, new Fallbacks());
	auto fwd1 = add(*fallbacks, withWork(new ForwardMockup(PredefinedCosts::constant(1.0)), 20ms));
	auto fwd2 = add(*fallbacks, withWork(new ForwardMockup(PredefinedCosts::constant(2.0)), 20ms));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ElementsAre(1, 1, 1));
	EXPECT_EQ(fwd1->runs_, 3u);
	EXPECT_EQ(fwd2->runs_, 0u);
}

TEST_F(ParallelFallbacks, PropagatorTriesNextAfterFailure) {
	add(t, new GeneratorMockup({ 0, 0, 0 }));
	auto fallbacks = add(t, new Fallbacks());
	auto fwd1 = add(*fallbacks, withWork(new ForwardMockup(PredefinedCosts::constant(INF)), 20ms));
	auto fwd2 = add(*fallbacks, withWork(new ForwardMockup(PredefinedCosts::constant(1.0)), 20ms));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ElementsAre(1, 1, 1));
	EXPECT_EQ(fwd1->runs_, 3u);
	EXPECT_EQ(fwd2->runs_, 3u);
}

TEST_F(ParallelFallbacks, GeneratorStaysWithFirstSuccessful) {
	auto fallbacks = add(t, new Fallbacks());
	auto gen1 = add(*fallbacks, withWork(new GeneratorMockup({ 1 }), 20ms));
	auto gen2 = add(*fallbacks, withWork(new GeneratorMockup({ 2 }), 20ms));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ElementsAre(1));
	EXPECT_EQ(gen1->runs_, 1u);
	EXPECT_EQ(gen2->runs_, 0u);
}

// a failed pair is handed to the next child, also if another child was picked for compute() before the failure
// got committed
TEST_F(ParallelFallbacks, ConnectPassesFailedPairToNextChild) {
	add(t, new GeneratorMockup({ 0, 0 }));
	auto fallbacks = add(t, new Fallbacks());
	auto con1 = add(*fallbacks, withWork(new ConnectMockup(PredefinedCosts::constant(INF)), 20ms));
	auto con2 = add(*fallbacks, withWork(new ConnectMockup(PredefinedCosts::constant(1.0)), 20ms));
	add(t, new GeneratorMockup({ 0, 0 }));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ElementsAre(1, 1, 1, 1));
	EXPECT_EQ(con1->runs_, 4u);
	EXPECT_EQ(con2->runs_, 4u);
}

// a wrapper's own compute() runs on the planning thread, while the jobs of its child run on workers
struct ParallelWrappers : TaskTestBase
{
	ParallelWrappers() { t.setNumThreads(4); }
};

TEST_F(ParallelWrappers, DelayingWrapperForwardsChildJobs) {
	add(t, new GeneratorMockup({ 0, 0, 0, 0, 0 }));
	auto child = Stage::pointer(withWork(new ForwardMockup(PredefinedCosts::constant(1.0)), 10ms));
	add(t, new DelayingWrapper({ 1, 0, 2 }, std::move(child)));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ElementsAre(1, 1, 1, 1, 1));
}

// stages declaring the same exclusive resource never compute at the same time
struct ExclusiveResources : TaskTestBase
{
	ExclusiveResources() { t.setNumThreads(4); }
};

namespace {
// GEN{0} → Alternatives{FWD(resource1), FWD(resource2)}, both stages simulating work
void buildTwoResourceUsers(Task& task, const void* resource1, const void* resource2, ConcurrencyProbe& probe) {
	task.add(Stage::pointer(new GeneratorMockup({ 0 })));
	auto alternatives = std::make_unique<Alternatives>();
	for (const void* resource : { resource1, resource2 }) {
		auto fwd = new ForwardMockup();
		fwd->resource = resource;
		alternatives->add(Stage::pointer(withWork(fwd, 50ms, &probe)));
	}
	task.add(std::move(alternatives));
}
}  // namespace

TEST_F(ExclusiveResources, SharedResourceNeverOverlaps) {
	int r;
	ConcurrencyProbe probe;
	buildTwoResourceUsers(t, &r, &r, probe);

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.numSolutions(), 2u);
	EXPECT_EQ(probe.max.load(), 1);
}

TEST_F(ExclusiveResources, DistinctResourcesOverlap) {
	int r1, r2;
	ConcurrencyProbe probe;
	buildTwoResourceUsers(t, &r1, &r2, probe);

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.numSolutions(), 2u);
	EXPECT_GE(probe.max.load(), 2);
}

TEST_F(ExclusiveResources, ClearedOnReinit) {
	int r;
	add(t, new GeneratorMockup({ 0 }));
	auto fwd = add(t, new ForwardMockup());

	fwd->resource = &r;
	t.init();
	EXPECT_THAT(fwd->pimpl()->exclusiveResources(), UnorderedElementsAre(&r));

	fwd->resource = nullptr;
	t.reset();
	t.init();
	EXPECT_THAT(fwd->pimpl()->exclusiveResources(), IsEmpty());
}

// When plan() stops early, the inputs of jobs that did not get committed are put back: a later plan() continues
// where the previous one stopped and finds the same solutions as sequential planning.
struct ResumePlanning : TaskTestBase
{
	ResumePlanning() { t.setNumThreads(4); }
};

namespace {
// GEN{0} → Alternatives{3 × FWD c1}: the FWDs share an exclusive resource,
// so while one of them computes, the jobs of the others are queued
// (The FWDs have equal costs: a resumed plan() changing the priority of the state, which Alternatives received in the
// previous plan(), would crash - also with sequential planning - as Task::init() replaces the container's interfaces.)
std::vector<ForwardMockup*> buildQueuedPropagators(Task& task, const void* resource) {
	std::vector<ForwardMockup*> fwds;
	task.add(Stage::pointer(new GeneratorMockup({ 0 })));
	auto alternatives = std::make_unique<Alternatives>();
	for (int i = 0; i < 3; ++i) {
		auto fwd = withWork(new ForwardMockup(PredefinedCosts::constant(1.0)), 50ms);
		fwd->resource = resource;
		alternatives->add(Stage::pointer(fwd));
		fwds.push_back(fwd);
	}
	task.add(std::move(alternatives));
	return fwds;
}

// GEN{0,0} → CON c1 → GEN{0} → CON c1 → GEN{0}: the CONs share an exclusive resource,
// so while one of them computes, the job of the other one is queued
std::vector<ConnectMockup*> buildQueuedConnectors(Task& task, const void* resource) {
	std::vector<ConnectMockup*> cons;
	task.add(Stage::pointer(new GeneratorMockup({ 0, 0 })));
	for (int i = 0; i < 2; ++i) {
		auto con = withWork(new ConnectMockup(PredefinedCosts::constant(1.0)), 50ms);
		con->resource = resource;
		task.add(Stage::pointer(con));
		cons.push_back(con);
		task.add(Stage::pointer(new GeneratorMockup({ 0 })));
	}
	return cons;
}

template <typename S>
std::size_t totalRuns(const std::vector<S*>& stages) {
	std::size_t runs = 0;
	for (const S* stage : stages)
		runs += stage->runs_;
	return runs;
}

// number of solutions found by sequential planning, stopped by plan(1) and resumed by plan()
template <typename Build>
std::size_t numSolutionsOfSequentialResume(Build build) {
	int resource;
	Task task;
	setup(task, 1);
	build(task, &resource);
	EXPECT_TRUE(task.plan(1));
	EXPECT_TRUE(task.plan());
	return task.numSolutions();
}
}  // namespace

TEST_F(ResumePlanning, PropagatorInputsOfCancelledJobsArePutBack) {
	int r;
	const auto fwds = buildQueuedPropagators(t, &r);

	EXPECT_TRUE(t.plan(1));  // stops after the first job: queued jobs get cancelled
	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ElementsAre(1, 1, 1));
	EXPECT_EQ(totalRuns(fwds), 3u);  // each input was computed exactly once
	EXPECT_EQ(t.numSolutions(), numSolutionsOfSequentialResume(buildQueuedPropagators));
}

TEST_F(ResumePlanning, ConnectorPairsOfCancelledJobsArePutBack) {
	int r;
	const auto cons = buildQueuedConnectors(t, &r);
	// preempt as soon as both CONs have a job: one of them runs, the other one is queued (and gets cancelled)
	bool preempted = false;
	t.addTaskCallback([this, &cons, &preempted](const Task& /*task*/) {
		if (!preempted && cons[0]->pimpl()->busy() && cons[1]->pimpl()->busy()) {
			preempted = true;
			t.preempt();
		}
	});

	EXPECT_EQ(t.plan().val, MoveItErrorCode::PREEMPTED);
	ASSERT_TRUE(preempted);
	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ElementsAre(2, 2));
	EXPECT_EQ(totalRuns(cons), 3u);  // each pair was computed exactly once
	EXPECT_EQ(t.numSolutions(), numSolutionsOfSequentialResume(buildQueuedConnectors));
}

// An exception thrown on the planning thread (here by a solution callback) aborts planning: plan() throws once
// the running jobs finished, their results are discarded and their inputs put back for a later plan().
TEST_F(ResumePlanning, AbortedJobsPutTheirInputsBack) {
	// GEN{0,0} → FWD1 c1 → FWD2 c2: FWD1 computes its 2nd state while FWD2 completes the first solution
	add(t, new GeneratorMockup({ 0, 0 }));
	auto fwd1 = add(t, new ForwardMockup(PredefinedCosts::constant(1.0)));
	auto fwd2 = add(t, withWork(new ForwardMockup(PredefinedCosts::constant(2.0)), 10ms));
	std::atomic<bool> slow_finished{ false };
	fwd1->on_compute = [fwd1, &slow_finished] {
		if (fwd1->runs_ != 1) {  // 1st and 3rd run
			std::this_thread::sleep_for(10ms);
			return;
		}
		std::this_thread::sleep_for(200ms);
		slow_finished = true;
	};
	bool thrown = false;
	t.addSolutionCallback([&thrown](const SolutionBase& /*s*/) {
		if (!thrown) {
			thrown = true;
			throw std::runtime_error("callback failed");
		}
	});

	EXPECT_THROW(t.plan(), std::runtime_error);  // thrown when the first solution reaches the task
	EXPECT_TRUE(slow_finished);  // plan() returned only after the running job finished
	EXPECT_FALSE(t.stages()->pimpl()->busy());
	EXPECT_COSTS(t.solutions(), ElementsAre(3));  // the result of FWD1's slow job was discarded

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ElementsAre(3, 3));
	EXPECT_EQ(fwd1->runs_, 3u);  // the input of the discarded job was computed again
	EXPECT_EQ(fwd2->runs_, 2u);
}

namespace {
// planner without any functionality, relying on the default exclusiveResources()
struct StubPlanner : solvers::PlannerInterface
{
	void init(const moveit::core::RobotModelConstPtr& /*robot_model*/) override {}

	Result plan(const planning_scene::PlanningSceneConstPtr& /*from*/,
	            const planning_scene::PlanningSceneConstPtr& /*to*/, const moveit::core::JointModelGroup* /*jmg*/,
	            double /*timeout*/, robot_trajectory::RobotTrajectoryPtr& /*result*/,
	            const moveit_msgs::msg::Constraints& /*path_constraints*/) override {
		return { false, "stub" };
	}

	Result plan(const planning_scene::PlanningSceneConstPtr& /*from*/, const moveit::core::LinkModel& /*link*/,
	            const Eigen::Isometry3d& /*offset*/, const Eigen::Isometry3d& /*target*/,
	            const moveit::core::JointModelGroup* /*jmg*/, double /*timeout*/,
	            robot_trajectory::RobotTrajectoryPtr& /*result*/,
	            const moveit_msgs::msg::Constraints& /*path_constraints*/) override {
		return { false, "stub" };
	}

	std::string getPlannerId() const override { return "stub"; }
};
}  // namespace

struct PlannerResources : TaskTestBase
{};

TEST_F(PlannerResources, DefaultSerializesInstance) {
	StubPlanner p;
	EXPECT_THAT(p.exclusiveResources(), ElementsAre(&p));
}

TEST_F(PlannerResources, StatelessBuiltinsAreShareable) {
	EXPECT_THAT(solvers::JointInterpolationPlanner{}.exclusiveResources(), IsEmpty());
	EXPECT_THAT(solvers::CartesianPath{}.exclusiveResources(), IsEmpty());
}

TEST_F(PlannerResources, MultiPlannerCollectsChildren) {
	auto stub1 = std::make_shared<StubPlanner>();
	auto stub2 = std::make_shared<StubPlanner>();
	solvers::MultiPlanner planner{ stub1, std::make_shared<solvers::JointInterpolationPlanner>(), stub2 };
	EXPECT_THAT(planner.exclusiveResources(), UnorderedElementsAre(stub1.get(), stub2.get()));
}

// the pipeline is shared (and not necessarily reentrant), so planners using the same pipeline share the resource
TEST_F(PlannerResources, PipelinePlannersSharingAPipelineShareTheResource) {
	int dummy;  // non-owning pipeline pointer, which is never dereferenced
	planning_pipeline::PlanningPipelinePtr pipeline(std::shared_ptr<void>(),
	                                                reinterpret_cast<planning_pipeline::PlanningPipeline*>(&dummy));
	solvers::PipelinePlanner a(pipeline), b(pipeline);
	EXPECT_THAT(a.exclusiveResources(), ElementsAre(pipeline.get()));
	EXPECT_THAT(b.exclusiveResources(), ElementsAre(pipeline.get()));

	// without a pipeline (before init()), the planner itself is the resource
	solvers::PipelinePlanner c{ rclcpp::Node::SharedPtr() };
	EXPECT_THAT(c.exclusiveResources(), ElementsAre(&c));
}

TEST_F(PlannerResources, MoveToAndConnectRegisterSharedPlanner) {
	auto stub = std::make_shared<StubPlanner>();
	add(t, new GeneratorMockup({ 0 }));
	auto move = add(t, new stages::MoveTo("move", stub));
	auto rel = add(t, new stages::MoveRelative("rel", stub));
	auto con = add(t, new stages::Connect("con", { { "group", stub } }));
	add(t, new GeneratorMockup({ 0 }));

	t.init();
	EXPECT_THAT(move->pimpl()->exclusiveResources(), ElementsAre(stub.get()));
	EXPECT_THAT(rel->pimpl()->exclusiveResources(), ElementsAre(stub.get()));
	EXPECT_THAT(con->pimpl()->exclusiveResources(), ElementsAre(stub.get()));
}
