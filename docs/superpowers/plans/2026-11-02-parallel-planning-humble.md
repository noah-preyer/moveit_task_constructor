# Parallel Planning (ROS 2 Humble) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On the `humble` branch of MoveIt Task Constructor, let `Task::plan()` run the `compute()` calls of independent stages (generators, propagators, connectors) on a pool of worker threads, so that expensive motion planning (OMPL, Pilz, Cartesian, …) of different stages and alternatives overlaps in time. The results must match sequential planning.

**Target:** ROS 2 Humble only, starting from the fork's `humble` branch (`7566349`, identical to upstream `humble`). Jazzy comes later (see Out of Scope). Rolling is not a target.

**Architecture:** *Compute in parallel, commit serially.* The planning thread (the one calling `plan()`) keeps sole ownership of the solution graph: interfaces, `InterfaceState` links, container bookkeeping, pruning, callbacks and introspection. For each compute stage it takes one unit of work from the stage's input interface on the planning thread, then runs the user's `compute()` on a worker. Every `spawn`/`sendForward`/`sendBackward`/`connect`/`silentFailure` that runs on a worker is recorded instead of applied. When the job finishes, the planning thread replays those records. A stage has at most one job in flight. Two jobs can be stopped from running at the same time when they declare the same *exclusive resource*, for example a shared non-thread-safe planner.

**Tech Stack:** C++17 (set by Humble's `moveit_package()`; CI's clang-tidy runs with `-std=c++17`), `std::thread`/`std::mutex`/`std::condition_variable` (no new dependencies), ROS 2 Humble, MoveIt 2 Humble, gtest/gmock via `ament_cmake_gtest`/`ament_cmake_gmock`, pybind11 `smart_holder` branch (git submodule `core/python/pybind11`).

**Spec:** There is no separate spec. § Design below is the spec. Executors read it before starting any task.

**Environment:** The `moveit/moveit2:humble-source` docker image that the branch's CI uses. MoveIt is built from source there at `/root/ws_moveit/install`. Clone this repo with `--recurse-submodules` (pybind11 and scope_guard are submodules) to `<ws>/src/moveit_task_constructor`, then check out `humble`. Run every command from `<ws>` after `source /opt/ros/humble/setup.bash && source /root/ws_moveit/install/setup.bash`.

```bash
colcon build --packages-up-to moveit_task_constructor_core --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

```bash
colcon test --packages-select moveit_task_constructor_core --ctest-args -R <regex> && colcon test-result --verbose
```

In the tasks below, "Run `<regex>`" means: rebuild, then run the `colcon test` command above with that regex. A single gtest case can also be run directly after `source install/setup.bash`, e.g.
`build/moveit_task_constructor_core/test/moveit_task_constructor_core-test-parallel --gtest_filter='ParallelPlanning.*'`.

## Global Constraints

- **Default behavior does not change:** `Task::numThreads()` defaults to `1`. With `1`, `plan()` runs today's sequential loop, and every existing test passes without modification.
- **No new third-party dependencies.** Threads come from the C++ standard library only.
- **C++17 only.** No designated initializers, `std::jthread`, `std::stop_token`, `std::span`, concepts or `std::format`. Build `Job` objects by assigning members.
- **Humble header spelling.** MoveIt includes use `.h` (e.g. `<moveit/planning_scene/planning_scene.h>`). The new files in this plan need no MoveIt headers beyond those that existing MTC headers already pull in.
- **Source compatibility for user stages:** existing `Generator`, `MonitoringGenerator`, `PropagatingEitherWay`, `Connecting` and container subclasses compile unchanged. ABI breaks (new virtual functions and members) are acceptable for the next minor release, and the PR description must say so.
- **All solution-graph changes happen on the planning thread.** This covers `Interface::add/remove`, `InterfaceState` status and priority, `states_`, `solutions_`, `failures_`, `num_failures_`, container maps, `setStatus`, `liftSolution`, solution callbacks, task callbacks and introspection. No mutexes are added to these structures.
- **At most one in-flight job per stage.**
- **Worker exceptions never escape a worker.** `plan()` returns or throws only after every worker has stopped working on that task.
- **Private headers** (`*_p.h`) stay out of the install (existing `PATTERN "*_p.h" EXCLUDE`).
- **Formatting:** run `pre-commit run --files <changed files>` (clang-format, repo config) before every commit.
- **Commits** end with the session attribution trailer given by the harness. Push only to the designated branch.

## Review Focus

1. **One planning pipeline used by several stages.** This covers the tutorial pattern (one `PipelinePlanner` for every `MoveTo` and `Connect`). On Humble it also covers *separate* `PipelinePlanner` instances with the same pipeline name, because `PipelinePlanner::create()` hands all of them the same cached `PlanningPipeline`. Expected: calls into one pipeline never overlap, and results equal sequential planning. Pinned by `PlannerResources.MoveToAndConnectRegisterSharedPlanner`, `PlannerResources.PipelinePlannersSharingAPipelineShareTheResource` and `ExclusiveResources.SharedResourceNeverOverlaps` (Task 5).
2. **Preempt, timeout or a stage exception while jobs are in flight.** Expected: `plan()` returns `PREEMPTED`/`TIMED_OUT` or rethrows only after running jobs finished. Afterwards no stage reports `busy()`, and a later `plan()` works. Pinned by `ParallelPlanning.Preempt`, `.Timeout`, `.WorkerExceptionIsRethrownAfterOthersFinish` (Task 3).
3. **A `MonitoringGenerator` receives upstream solutions while its own `compute()` runs.** This is the pick pipeline: `GenerateGraspPose` monitoring `CurrentState`. Expected: no lost, duplicated or racy upstream solutions. Pinned by `ParallelPlanning.MonitoringGeneratorReceivesSolutionsWhileBusy` (Task 3).
4. **`Fallbacks` whose current child is still computing.** Expected: the container must not move on to the next child before the current child's results have been committed. Pinned by `ParallelFallbacks.*` (Task 4).
5. **Python-defined stages with `num_threads > 1`.** Expected: no GIL deadlock, because `Task.plan` releases the GIL and trampolines re-acquire it. Pinned by `test_parallel_*` in `rostest_trampoline.py` and `test_num_threads_property` (Task 7).

---

## Design

### What runs where

| Planning thread (owner of the graph) | Worker threads |
|---|---|
| `Task::plan` loop, `canCompute()` of all stages (including user `Generator::canCompute`) | `Generator::compute()` |
| Container `compute()` logic (Serial, Alternatives, Fallbacks, Merger, Wrapper incl. `ComputeIK`'s own IK) | `PropagatingEitherWay::computeForward/Backward()` (hence `MoveTo`, `MoveRelative`, `ModifyPlanningScene`, …) |
| Job preparation: taking the input state/pair off the interface, `performInitFrom(INTERFACE)` | `Connecting::compute(from, to)` (hence `Connect`) |
| Replaying recorded `spawn`/`send*`/`connect`/`silentFailure` (storeSolution, interfaces, lifting, pruning) | `CostTerm` evaluation for solutions created by the job (`computeCost` runs in the worker before recording) |
| `MonitoringGenerator::onNewSolution`, `Connecting::compatible`, solution and task callbacks, introspection | Planner calls (`PlannerInterface::plan`) |

### Rules for code running on a worker (documented for stage authors)

The worker side of a stage may:

- read its own stage's members and properties;
- read `InterfaceState::scene()` and `InterfaceState::properties()` of its inputs;
- call `spawn`, `sendForward`, `sendBackward`, `connect`, `silentFailure`, `storeFailures`, and planners.

It must not read `InterfaceState::priority()`, the incoming/outgoing trajectories of a state, other stages, or task state.

`Connecting::compatible()` may run on the planning thread while the same stage's `compute()` runs on a worker.

### Scheduling rules

1. `StagePrivate::runCompute()` checks whether a scheduler is attached and the stage `supportsJobs()` (compute stage with `parallelCompute()`). If so, it does not compute directly:
   - If the stage is busy, it returns.
   - If the stage can compute, it prepares one job, submits it, and returns.
   - Otherwise it falls through to the existing synchronous `compute()` call, which keeps today's behavior when `Alternatives`/`Merger` call `runCompute()` on an idle child that cannot compute.
2. While a leaf has a job in flight it is *busy*. Its private `canCompute()` returns `false`, so every container's existing "if canCompute → runCompute" logic skips it automatically. `inflight_` counts in-flight jobs in a stage's subtree, so a container is `busy()` while any descendant is.
3. A job's *prepare* step runs on the planning thread and extracts exactly what one sequential `compute()` call would consume:
   - propagator: one state from the single active pull interface, plus `performInitFrom(INTERFACE)`;
   - connector: one pair, via `pending.pop()`;
   - generator: nothing.
4. Workers take the **first queued job whose exclusive resources are all free**. A job holds its resources only while its `work` runs. Jobs are queued FIFO, at most one per stage, so no stage can starve another.
5. Job completion is processed on the planning thread in finish order:
   1. mark the stage idle (`inflight_ -= 1` on the stage and its ancestors) and add the job duration to their compute time;
   2. deliver notifications that were deferred while the stage was busy, in arrival order;
   3. replay the recorded commits in order;
   4. if the job threw `Property::error`, convert it with `reportPropertyError`.
6. Notifications aimed at a busy stage are queued, not delivered (today only `MonitoringGenerator` solution callbacks). Interface additions to a busy stage's pull interface stay immediate, because the worker no longer touches that interface.
7. Stopping:
   - On `max_solutions`, timeout or preempt: stop dispatching, **cancel queued jobs** (their prepared inputs are dropped, and their deferred notifications are delivered), **wait for running jobs and commit them**, then return. Overrun is bounded by the longest running job. The solution count may exceed `max_solutions` by up to the number of running jobs.
   - On an exception: same as above, then rethrow the first exception.
   - On an exception raised on the planning thread itself: abort, meaning cancel everything and discard uncommitted results.

### Exclusive resources

`ComputeBase::addExclusiveResource(const void*)` lets a stage declare, in `init()`, objects whose use must be serialized across stages. `PlannerInterface::exclusiveResources()` returns `{this}` by default: planners are assumed not thread-safe. `JointInterpolationPlanner` and `CartesianPath` return `{}` (stateless). `MultiPlanner` returns the union of its children's. `MoveTo`, `MoveRelative` and `Connect` register their planners' resources in `init()`.

**`PipelinePlanner` on Humble** is the single-pipeline version. `init()` gets its `planning_pipeline::PlanningPipeline` from `PipelinePlanner::create()`. That function keeps a process-wide static cache keyed by (robot model, pipeline namespace, adapter parameter), so every `PipelinePlanner("ompl")` shares one pipeline object. Some planner plugins are not reentrant. So `PipelinePlanner::exclusiveResources()` returns `{ planner_.get() }`, the shared pipeline, or `{this}` before `init()`. Pipelines with different namespaces (e.g. `ompl` and `pilz_industrial_motion_planner`) run in parallel.

To plan OMPL in parallel on Humble, users construct separate pipelines and pass them in: `std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "ompl", "planning_plugin", "request_adapters")` → `PipelinePlanner(pipeline)`.

### Known limits (documented, not fixed here)

- Container-internal work stays on the planning thread: `ComputeIK`'s IK, `Merger`'s trajectory merging.
- Kinematics plugins are assumed thread-safe. MoveIt makes the same assumption for parallel OMPL attempts. A plugin that isn't needs `setParallelCompute(false)` on the affected stages.
- The order in which solutions are found is not deterministic.
- The existing race between the introspection `get_solution` service thread and planning is unchanged.

## File Structure

| File | Responsibility |
|---|---|
| `core/include/moveit/task_constructor/job_scheduler_p.h` (new, private) | `Job` and `JobScheduler`. Generic: no knowledge of stages. |
| `core/src/job_scheduler.cpp` (new) | Worker pool, resource-aware queue, finish queue, inline mode, drain/abort |
| `core/include/moveit/task_constructor/stage_p.h` | Job hooks on `StagePrivate`, prepare overrides on the three leaf privates, `ComputeBasePrivate` resources |
| `core/src/stage.cpp` | `runCompute` job path, commit/defer split of `send*`/`spawn`/`connect`, leaf `prepareJob`s, MonitoringGenerator inbox |
| `core/include/moveit/task_constructor/stage.h` | `Stage::setParallelCompute/parallelCompute`, `ComputeBase::init/addExclusiveResource` |
| `core/include/moveit/task_constructor/task.h`, `task_p.h`, `core/src/task.cpp` | `setNumThreads/numThreads`, `planSequential`, `planParallel`, scheduler attach, inline test hook |
| `core/src/container.cpp` | Busy-aware `Fallbacks` |
| `core/include/moveit/task_constructor/solvers/*.h`, `core/src/solvers/multi_planner.cpp`, `core/src/solvers/pipeline_planner.cpp` | `PlannerInterface::exclusiveResources()` and overrides (Humble `PipelinePlanner`: the shared pipeline) |
| `core/src/stages/{move_to,move_relative,connect,generate_random_pose}.cpp`, `stages/generate_random_pose.h` | Resource registration, per-instance RNG |
| `core/python/bindings/src/core.cpp` | `Task.num_threads`, `Stage.parallel_compute`, GIL release in `Task.plan` |
| `core/test/stage_mockups.{h,cpp}` | `ConcurrencyProbe`, `on_compute` hooks, `resource` on propagator mockup, `QueueingMonitoringGeneratorMockup`, `MTC_TEST_JOB_MODE` hook |
| `core/test/test_job_scheduler.cpp`, `core/test/test_parallel.cpp`, `core/test/test_generate_random_pose.cpp` (new) | Tests |
| `core/doc/howto.rst`, `.github/workflows/ci.yaml`, `.github/workflows/tsan.suppressions` | Docs, ThreadSanitizer CI job |

---

### Task 1: Generic job scheduler

**Files:**
- Create: `core/include/moveit/task_constructor/job_scheduler_p.h`, `core/src/job_scheduler.cpp`
- Modify: `core/src/CMakeLists.txt` (add `${PROJECT_INCLUDE}/job_scheduler_p.h` and `job_scheduler.cpp` to `add_library(${PROJECT_NAME} …)`)
- Modify: `core/test/stage_mockups.h/.cpp` (add `ConcurrencyProbe`)
- Test: `core/test/test_job_scheduler.cpp`. Register it with `mtc_add_gtest(test_job_scheduler.cpp)` in `core/test/CMakeLists.txt`.

**Interfaces:**
- Consumes: nothing.
- Produces (namespace `moveit::task_constructor`):

```cpp
struct Job {
	const void* owner = nullptr;                          // opaque id of the submitter (the StagePrivate*)
	std::function<void()> work;                           // runs on a worker (inline mode: calling thread)
	std::vector<const void*> resources;                   // held exclusively while `work` runs
	std::function<void(Job&)> complete;                   // planning thread, after `work` finished
	std::function<void(Job&)> cancel;                     // planning thread, for jobs discarded by drain()/abort()
	std::vector<std::function<void()>> commits;           // filled through defer() during `work`
	std::exception_ptr error;                             // set if `work` threw
	std::chrono::duration<double> duration{};             // wall time of `work`
	void defer(std::function<void()>&& commit) { commits.push_back(std::move(commit)); }
	static Job* current();                                // job whose `work` runs on this thread, else nullptr
};

class JobScheduler {
public:
	explicit JobScheduler(unsigned int num_threads);      // 0 = inline mode
	~JobScheduler();                                      // abort(), then join workers
	void submit(Job&& job);                               // inline mode: run work, run complete, rethrow job.error
	std::size_t inflight() const;                         // submitted, not yet completed/cancelled
	bool waitForFinished(std::chrono::steady_clock::time_point deadline);  // true if ≥1 finished job is waiting
	std::size_t commitFinished();                         // run `complete` for finished jobs (finish order); returns count; never throws job errors
	std::exception_ptr error() const;                     // first job.error, read *after* its `complete` ran
	void drain();                                         // cancel queued, wait running, commitFinished()
	void abort() noexcept;                                // cancel queued, wait running, `cancel` (not complete) for all of them
};

struct ConcurrencyProbe {                                 // test helper (stage_mockups.h)
	std::atomic<int> current{ 0 }, max{ 0 };
	struct Scope { explicit Scope(ConcurrencyProbe& p); ~Scope(); ConcurrencyProbe& probe; };  // ++current, update max / --current
};
```

- [ ] **Step 1: Write the failing tests** in `test_job_scheduler.cpp`:
  - `JobScheduler.InlineRunsWorkAndCompleteSynchronously`: `JobScheduler s(0)`. The work records `Job::current() != nullptr`, and complete sets a flag. Right after `submit`: `EXPECT_TRUE(ran_inside_job)`, `EXPECT_TRUE(completed)`, `EXPECT_EQ(s.inflight(), 0u)`.
  - `JobScheduler.InlineRethrowsAfterComplete`: the work throws `std::runtime_error`. `EXPECT_THROW(s.submit(...), std::runtime_error)` and `EXPECT_TRUE(completed)`.
  - `JobScheduler.CurrentIsNullOutsideWork`: `EXPECT_EQ(Job::current(), nullptr)` on the test thread.
  - `JobScheduler.DeferredCommitsReplayedOnCallingThread`: `JobScheduler s(2)`. The work calls `Job::current()->defer(...)` twice, pushing 1 and 2. `complete` runs `for (auto& c : job.commits) c();` and records its thread id. After `waitForFinished(now + 1s)` and `commitFinished() == 1`: `EXPECT_THAT(values, ElementsAre(1, 2))` and the complete thread id equals the test thread id.
  - `JobScheduler.RunsJobsConcurrently`: `JobScheduler s(4)`. Submit 4 jobs, each holding `ConcurrencyProbe::Scope` for 50 ms. Loop `waitForFinished` + `commitFinished` until 4 completed, then `EXPECT_GE(probe.max, 2)`.
  - `JobScheduler.SharedResourceSerializesJobs`: same, but every job has `resources = {&r}`. `EXPECT_EQ(probe.max, 1)` and 4 completed.
  - `JobScheduler.BlockedJobDoesNotBlockOthers`: `JobScheduler s(2)`. Submit A `{&r}` sleeping 100 ms, then B `{&r}` sleeping 10 ms, then C `{}` sleeping 10 ms. Record finish order in `complete`, then `EXPECT_THAT(order, ElementsAre('C', 'A', 'B'))`.
  - `JobScheduler.ErrorIsRecordedNotThrown`: `JobScheduler s(2)`, the work throws. `EXPECT_NO_THROW(s.commitFinished())`, `EXPECT_TRUE(s.error())` and `EXPECT_TRUE(completed)`.
  - `JobScheduler.DrainCancelsQueuedAndCommitsRunning`: `JobScheduler s(1)`. A sleeps 50 ms, B and C are queued. Wait until A started (atomic flag), then `s.drain()`. A completed, B and C cancelled, `inflight() == 0`.
  - `JobScheduler.AbortDiscardsRunning`: `JobScheduler s(1)`. A sleeps 50 ms and has started; call `s.abort()`. A's `cancel` was called, `complete` was not, and `inflight() == 0`.
  - `JobScheduler.WaitForFinishedHonorsDeadline`: the job sleeps 200 ms. `waitForFinished(now + 20ms)` returns `false` within 150 ms.
- [ ] **Step 2: Run `test-job-scheduler`.** Expected: build failure, "job_scheduler_p.h: No such file".
- [ ] **Step 3: Implement `Job` and `JobScheduler`** in the files above.
  - Keep jobs as `std::unique_ptr<Job>` in a `std::list` (queued), a running count, and a `std::deque` (finished). Track `std::set<const void*> held_resources_`. Use one mutex, `cv_work_` for workers and `cv_finished_` for the planning thread. `Job::current()` is a `thread_local Job*` set around `work`.
  - A worker scans the queue for the first job whose resources are disjoint from `held_resources_`. If none, it waits on `cv_work_`. When the work finishes, the worker releases the resources, moves the job to finished, and calls `notify_all` on both condition variables.
  - The worker catches every exception into `job.error` and measures `duration` around `work`.
  - `commitFinished` pops each finished job, calls `complete(job)`, then records `job.error` into `error()` if none is recorded yet.
  - In inline mode, `submit` does work, then complete, then rethrows `job.error`, all on the caller's thread.
- [ ] **Step 4: Run `test-job-scheduler`.** Expected: all pass. Rerun 20×: `--gtest_repeat=20` on the binary. Expected: still all pass.
- [ ] **Step 5: Commit:** `git add core/include/moveit/task_constructor/job_scheduler_p.h core/src/job_scheduler.cpp core/src/CMakeLists.txt core/test/test_job_scheduler.cpp core/test/stage_mockups.* core/test/CMakeLists.txt && git commit -m "Add JobScheduler for parallel stage computation"`

---

### Task 2: Job path for compute stages (validated in inline mode)

**Files:**
- Modify: `core/include/moveit/task_constructor/stage_p.h`, `core/src/stage.cpp`, `core/include/moveit/task_constructor/stage.h`
- Modify: `core/include/moveit/task_constructor/task_p.h`, `core/src/task.cpp`
- Modify: `core/test/stage_mockups.h/.cpp`, `core/test/CMakeLists.txt`
- Test: `core/test/test_parallel.cpp` (new, `mtc_add_gmock(test_parallel.cpp)`)

**Interfaces:**
- Consumes: `Job`, `JobScheduler` (Task 1).
- Produces:

```cpp
// StagePrivate (stage_p.h)
void setScheduler(JobScheduler* scheduler);
bool busy() const;                                        // inflight_ > 0 (own job or any descendant's)
virtual bool supportsJobs() const { return false; }       // ComputeBasePrivate: return parallel_compute_;
virtual std::function<void()> prepareJob() { return {}; } // only called when supportsJobs() && canCompute()
virtual std::vector<const void*> exclusiveResources() const { return {}; }
void deferWhileBusy(std::function<void()>&& notification);  // queue if busy() && scheduler attached, else call now
void runCompute();                                        // now out-of-line in stage.cpp
// private: submitJob(), onJobFinished(Job&), onJobCancelled(Job&), addInflight(int), addComputeTime(duration),
//          commitOrDefer(std::function<void()>&&), commitForward/commitBackward/commitSpawn/commitConnect(...)
// members: JobScheduler* scheduler_ = nullptr; unsigned int inflight_ = 0;
//          std::deque<std::function<void()>> deferred_notifications_; bool parallel_compute_ = true;

// Stage (stage.h, public)
void setParallelCompute(bool enable);  // default true; only Generator/Propagating*/Connecting stages honor it
bool parallelCompute() const;

// TaskPrivate (task_p.h)
void setInlineJobs(bool enable);       // test hook: plan() runs the job path with JobScheduler(0)
void attachScheduler(JobScheduler* s); // setScheduler(s) on every stage of traverseStages(…, 1, UINT_MAX)

// stage_mockups.h: TaskTestBase reads env MTC_TEST_JOB_MODE; "inline" → t.pimpl()->setInlineJobs(true)
```

- [ ] **Step 1: Write the failing tests** in `test_parallel.cpp`. Use a fixture `InlineJobs : TaskTestBase` that calls `t.pimpl()->setInlineJobs(true)`. Mockups get `std::function<void()> on_compute;`, invoked first thing in `GeneratorMockup::compute`, `PropagatorMockup::computeForward/Backward` and `ConnectMockup::compute`.
  - `InlineJobs.ComputeRunsInsideJob`: GEN{0} → FWD whose `on_compute` records `Job::current() != nullptr`. Expect `true`, and `t.numSolutions() == 1`.
  - `InlineJobs.DisabledParallelComputeRunsDirectly`: same, with `fwd->setParallelCompute(false)`. Expect `false`.
  - `InlineJobs.SilentFailureIsCounted`: a `ForwardMockup` subclass whose `computeForward` only calls `silentFailure()`. After `t.plan()`: `EXPECT_EQ(fwd->numFailures(), 1u)`.
  - `InlineJobs.PropertyErrorStillReported`: FWD `on_compute` calls `fwd->properties().get<double>("undeclared")`. `t.plan()` throws `std::runtime_error` whose `what()` contains `"in stage '" + fwd->name() + "'"`.
  - `InlineJobs.WithoutInlineJobsNoJobIsUsed`: the same as `ComputeRunsInsideJob` without the hook. Expect `false`; this guards the `numThreads()==1` path.
- [ ] **Step 2: Run `test-parallel`.** Expected: compile failure (`setInlineJobs`, `Job` unknown in tests).
- [ ] **Step 3: Split commit from compute in `StagePrivate`.** `sendForward`, `sendBackward`, `spawn(from,to)` and `connect` keep their asserts, call `computeCost(...)` immediately (on the worker when in a job), then pass the rest of the old body as a closure to `commitOrDefer`, each to its own `commit*` function. `commitOrDefer` calls `Job::current()->defer()` when `Job::current() && Job::current()->owner == this`. Otherwise it runs the closure now. Assert that `Job::current()` is null or owned by `this`. `Stage::silentFailure()` routes `++num_failures_` through `commitOrDefer`. Copy `parallel_compute_` in `StagePrivate::operator=`.
- [ ] **Step 4: Implement the job path.**
  - `runCompute` follows Design rule 1 exactly; the legacy body is unchanged.
  - `submitJob()` builds the `Job`: owner, `prepareJob()`, `exclusiveResources()`, complete/cancel bound to `onJobFinished` / `onJobCancelled`. It calls `addInflight(+1)` and then submits.
  - `onJobFinished` follows Design rule 5 in order. For step 4, rethrow `job.error` inside `try`, catch `Property::error` and call `me()->reportPropertyError(e)`, then store the resulting exception back into `job.error`.
  - `onJobCancelled`: `addInflight(-1)`, then deliver deferred notifications.
  - `addInflight` and `addComputeTime` walk `parent()->pimpl()` up to the root.
  - The leaf privates `PropagatingEitherWayPrivate`, `ConnectingPrivate` and `GeneratorPrivate` get `canCompute()` that returns `false` when `busy()`, and `prepareJob()` per Design rule 3. The closure captures `me` and references to the stable `InterfaceState`s. After `resolveInterface`, exactly one of `starts_`/`ends_` exists for a propagator.
  - `MonitoringGeneratorPrivate::solutionCB` wraps the `onNewSolution` call in `deferWhileBusy`.
  - `Stage::reset()` clears `deferred_notifications_` and sets `inflight_ = 0`.
- [ ] **Step 5: Implement the inline hook in `Task::plan`.** Move today's loop body unchanged into a private `Task::planSequential(size_t max_solutions)`. `plan()` calls `init()`. If `inline_jobs_`, it creates `JobScheduler scheduler(0)`, calls `attachScheduler(&scheduler)` with a scope guard that calls `attachScheduler(nullptr)`, then calls `planSequential`. Otherwise it calls `planSequential` directly.
- [ ] **Step 6: Run the existing suites in inline mode.** Humble's `ament_add_gtest_test()` has no `TEST_NAME` argument and always names the test after the target, which already exists. So register the reruns with the lower-level `ament_add_test`. Add to `core/test/CMakeLists.txt` (after the `mtc_add_*` calls):

  ```cmake
  function(mtc_add_inline_test name)  # rerun an existing gtest binary with MTC_TEST_JOB_MODE=inline
  	set(target ${PROJECT_NAME}-${name})
  	set(result ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${target}-inline.gtest.xml)
  	ament_add_test(${target}-inline
  		COMMAND $<TARGET_FILE:${target}> --gtest_output=xml:${result}
  		OUTPUT_FILE ${CMAKE_BINARY_DIR}/ament_cmake_gtest/${target}-inline.txt
  		RESULT_FILE ${result}
  		ENV MTC_TEST_JOB_MODE=inline)
  endfunction()
  foreach(name test-container test-serial test-pruning test-fallback)
  	mtc_add_inline_test(${name})
  endforeach()
  ```

  Implement the env check in the `TaskTestBase` constructor. `stage_mockups.h` includes `task_p.h`.
- [ ] **Step 7: Run `test-parallel|inline|test-container|test-serial|test-pruning|test-fallback`.** Expected: everything passes in both modes. If an inline run differs from its sequential twin, the decomposition is wrong (commit order, a missed deferral, or bookkeeping). Fix the code, not the test.
- [ ] **Step 8: Commit:** `git commit -m "Run compute stages as deferred-commit jobs when a scheduler is attached"`

---

### Task 3: Threaded planning loop and `Task::setNumThreads`

**Files:**
- Modify: `core/include/moveit/task_constructor/task.h`, `task_p.h`, `core/src/task.cpp`
- Modify: `core/test/stage_mockups.h/.cpp`:
  - add `QueueingMonitoringGeneratorMockup`;
  - add `std::function<void()> simulateWork(std::chrono::milliseconds duration, ConcurrencyProbe* probe = nullptr)`, which returns an `on_compute` that holds a `ConcurrencyProbe::Scope` (if `probe` is set) while sleeping `duration`;
  - change `ConnectMockup::compute` to stop reading `priority()` and set the comment to `std::to_string(runs_)`, per the worker rules.
- Test: `core/test/test_parallel.cpp`

**Interfaces:**
- Consumes: Task 1 and Task 2 (`attachScheduler`, `busy()`, `planSequential`).
- Produces:

```cpp
// Task (public)
void setNumThreads(unsigned int num_threads);  // 1 (default) = sequential; 0 = std::thread::hardware_concurrency()
unsigned int numThreads() const;               // returns the stored value (0 stays 0)
// Task (private)
moveit::core::MoveItErrorCode planParallel(size_t max_solutions, unsigned int num_threads);
// TaskPrivate: unsigned int num_threads_ = 1;  copied in TaskPrivate::operator=(TaskPrivate&&)
// Mockup: struct QueueingMonitoringGeneratorMockup : MonitoringGenerator
//   onNewSolution pushes &s to std::deque; canCompute = !empty; compute: on_compute(), pop, ++runs_,
//   spawn(InterfaceState(s.end()->scene()->diff()), SubTrajectory{})
```

- [ ] **Step 1: Write the failing tests** (fixture `ParallelPlanning : TaskTestBase`, which calls `t.setNumThreads(4)`):
  - `ParallelPlanning.RunsComputeConcurrently`: GEN{0} → Alternatives{4 × FWD c=0 with `simulateWork(50ms, &probe)`}. Expect 4 solutions and `probe.max >= 2`. With `t.setNumThreads(1)` on a fresh task: `probe.max == 1`.
  - `ParallelPlanning.MatchesSequentialResults`: build each topology 20× into a fresh `Task`. Every mockup has `simulateWork(2ms)`. Assert `EXPECT_COSTS(t.solutions(), ElementsAreArray(expected))`:
    - GEN{0,0,0} → FWD c1 → FWD c2 ⇒ {3,3,3}
    - GEN{0,0} → Alternatives{FWD c1, FWD c2, FWD c3} ⇒ {1,1,2,2,3,3}
    - GEN{0,0,0} → CON c1 → GEN{0,0} ⇒ six × 1
    - GEN{0,0} → Serial{FWD c1, Alternatives{FWD c1, FWD c2}} ⇒ {2,2,3,3}
  - `ParallelPlanning.ComputeOnWorkerCallbacksOnPlanningThread`: record thread ids in FWD `on_compute`, in `t.addTaskCallback` and in `t.addSolutionCallback`. The compute id differs from the test thread; both callback ids equal it.
  - `ParallelPlanning.Preempt`: GEN constant 0 (infinite) → FWD INF (failures only) with `simulateWork(20ms)`. Run `plan()` in a `std::thread`, sleep 100 ms, `t.preempt()`, join. Expect `PREEMPTED`, join within 500 ms of `preempt()`, and `EXPECT_FALSE(t.stages()->pimpl()->busy())`. Then `t.setTimeout(0.3); EXPECT_EQ(t.plan(), TIMED_OUT)`: a stale busy flag would end that run early with `PLANNING_FAILED`.
  - `ParallelPlanning.Timeout`: the same failing chain with `t.setTimeout(0.2)`. Expect `TIMED_OUT` and wall time < 0.5 s.
  - `ParallelPlanning.MaxSolutions`: GEN constant 0 → FWD c1 with `simulateWork(5ms)`. `t.plan(3)` returns `SUCCESS` and `3 <= numSolutions() <= 7`.
  - `ParallelPlanning.WorkerExceptionIsRethrownAfterOthersFinish`: GEN{0} → Alternatives{FWD whose `on_compute` waits (≤ 1 s) until `std::atomic<bool> started` is set and then throws `std::runtime_error("boom")`, FWD whose `on_compute` sets `started`, sleeps 100 ms, then sets `std::atomic<bool> finished`}. `EXPECT_THROW(t.plan(), std::runtime_error)`, then `EXPECT_TRUE(finished)` and `EXPECT_FALSE(t.stages()->pimpl()->busy())`.
  - `ParallelPlanning.MonitoringGeneratorReceivesSolutionsWhileBusy`: GEN{0,0,0,0,0} with `simulateWork(5ms)` → CON c0 → QMON (monitors GEN, `simulateWork(20ms)`). Repeat 10× on fresh tasks: `numSolutions() == 25` and `qmon->runs_ == 5`.
  - `ParallelPlanning.ComputeTimeIsAccumulated`: GEN{0,0,0} → FWD `simulateWork(20ms)` ⇒ `fwd->getTotalComputeTime() >= 0.06`.
  - `ParallelPlanning.NumThreadsSurvivesMove`: `Task a; a.setNumThreads(3); Task b(std::move(a));` ⇒ `b.numThreads() == 3u`.
  - `ParallelPlanning.ZeroMeansHardwareConcurrency`: `setNumThreads(0)`. `numThreads() == 0u` and the first topology above plans to {3,3,3}.
- [ ] **Step 2: Run `test-parallel`.** Expected: compile failure (`setNumThreads`).
- [ ] **Step 3: Implement `setNumThreads`/`numThreads`.** In `plan()`, after `init()`, resolve `n = num_threads_ == 0 ? max(1u, hardware_concurrency()) : num_threads_`. The inline hook keeps priority. Use `n == 1` → `planSequential`, otherwise `planParallel(max_solutions, n)`.
- [ ] **Step 4: Implement `planParallel`** (guard: `sg::make_scope_guard` → `scheduler.abort(); attachScheduler(nullptr)`; `success_or` as in `planSequential`):

```text
JobScheduler scheduler(n); attachScheduler(&scheduler); deadline = start + timeout(); result = PLANNING_FAILED
loop:
  if preempt_requested_            → result = PREEMPTED; break
  if now >= deadline               → result = TIMED_OUT; break
  if max_solutions && numSolutions() >= max_solutions → break
  progress = false
  if canCompute()                  → compute(); progress = true        // dispatches jobs / runs serial work
  else if scheduler.inflight()==0  → break                             // exhausted
  else scheduler.waitForFinished(min(deadline, now + 10ms))
  progress |= scheduler.commitFinished() > 0
  if scheduler.error()             → break
  if progress                      → task callbacks; introspection publishTaskState()
scheduler.drain(); attachScheduler(nullptr)          // planning-thread exceptions skip this and hit the guard
run task callbacks + publishTaskState once more if drain committed jobs
if scheduler.error() → rethrow it
return success_or(result)
```

- [ ] **Step 5: Run `test-parallel`, then `--gtest_repeat=50` on the binary.** Expected: all pass, every repetition.
- [ ] **Step 6: Run `.*`** (whole package). Expected: all pass. This includes the inline reruns.
- [ ] **Step 7: Commit:** `git commit -m "Add Task::setNumThreads and threaded planning loop"`

---

### Task 4: Containers under concurrency

**Files:**
- Modify: `core/src/container.cpp` (`FallbacksPrivateCommon::canCompute`, `FallbacksPrivateConnect::onNewFailure`)
- Test: `core/test/test_parallel.cpp`

**Interfaces:**
- Consumes: `StagePrivate::busy()` (Task 2), `Task::setNumThreads` (Task 3), mockup `on_compute`/`simulateWork`.
- Produces: no new API.

- [ ] **Step 1: Write the failing tests** (fixture `ParallelFallbacks : TaskTestBase` with `setNumThreads(4)`; every mockup inside `Fallbacks` gets `simulateWork(20ms)`):
  - `ParallelFallbacks.PropagatorDoesNotSkipBusyChild`: GEN{0,0,0} → Fallbacks{FWD c1, FWD c2}. Costs {1,1,1}, `fwd2->runs_ == 0`.
  - `ParallelFallbacks.PropagatorTriesNextAfterFailure`: GEN{0,0,0} → Fallbacks{FWD INF, FWD c1}. Costs {1,1,1}, `fwd1->runs_ == 3`, `fwd2->runs_ == 3`.
  - `ParallelFallbacks.GeneratorStaysWithFirstSuccessful`: Fallbacks{GEN{1}, GEN{2}} as the only task stage. Costs {1}, `gen2->runs_ == 0`.
  - `ParallelFallbacks.ConnectPassesFailedPairToNextChild`: GEN{0,0} → Fallbacks{CON INF, CON c1} → GEN{0,0}. Costs four × 1, `con1->runs_ == 4`, `con2->runs_ == 4`. In a Debug build, no assertion fires.
  - `ParallelWrappers.DelayingWrapperForwardsChildJobs`: GEN{0,0,0,0,0} → `DelayingWrapper({1, 0, 2}, FWD c1 with simulateWork(10ms))` (existing mockup). Expect five solutions, all cost 1. Humble has no `LimitSolutions` stage. This covers a wrapper whose own `compute()` runs on the planning thread while its child's jobs run on workers.
- [ ] **Step 2: Run `test-parallel`.** Expected: the first four fail (extra solutions or children computing too early). The wrapper test may already pass; keep it as a regression test.
- [ ] **Step 3: Make `FallbacksPrivateCommon::canCompute()` wait for a busy child.** If `current_ != children().end() && (*current_)->pimpl()->busy()`, return `false` before calling `nextJob()`.
- [ ] **Step 4: Make `FallbacksPrivateConnect::onNewFailure` independent of `active_`.** Find the next child as `std::next(child.pimpl()->it())`, not from `active_`, and drop the `active_ == &child` assert.
- [ ] **Step 5: Run `test-parallel|test-fallback`.** Expected: all pass, inline twins included. Rerun `test-parallel` with `--gtest_repeat=50`.
- [ ] **Step 6: Commit:** `git commit -m "Make Fallbacks aware of in-flight child jobs"`

---

### Task 5: Exclusive resources and planners

**Files:**
- Modify: `core/include/moveit/task_constructor/stage.h`, `stage_p.h`, `core/src/stage.cpp`
- Modify: `core/include/moveit/task_constructor/solvers/planner_interface.h`, `multi_planner.h`, `joint_interpolation.h`, `cartesian_path.h`, `pipeline_planner.h`, `core/src/solvers/multi_planner.cpp`, `core/src/solvers/pipeline_planner.cpp`
- Modify: `core/src/stages/move_to.cpp`, `move_relative.cpp`, `connect.cpp`
- Modify: `core/test/stage_mockups.h/.cpp` (`PropagatorMockup`: member `const void* resource = nullptr;`, plus an `init` override that calls `addExclusiveResource(resource)` when set)
- Test: `core/test/test_parallel.cpp`

**Interfaces:**
- Consumes: `StagePrivate::exclusiveResources()` (Task 2), `JobScheduler` resource handling (Task 1).
- Produces:

```cpp
// ComputeBase
void init(const moveit::core::RobotModelConstPtr& robot_model) override;  // clears resources, then Stage::init
protected: void addExclusiveResource(const void* resource);                 // call from init(); nullptr ignored
// ComputeBasePrivate: std::set<const void*> exclusive_resources_; exclusiveResources() returns them as a vector
// PlannerInterface
virtual std::vector<const void*> exclusiveResources() const { return { this }; }
// JointInterpolationPlanner, CartesianPath: override → {}
// MultiPlanner: override → concatenation of children's exclusiveResources()
// PipelinePlanner (Humble): override → { planner_.get() } once a pipeline is set, else { this }
```

- [ ] **Step 1: Write the failing tests** (`StubPlanner : solvers::PlannerInterface` defined in the test; `init` is a no-op, both `plan` overloads return `{false, "stub"}`, `getPlannerId` returns `"stub"`):
  - `ExclusiveResources.SharedResourceNeverOverlaps`: 4 threads. GEN{0} → Alternatives{FWD, FWD}, both with `resource = &r` and `simulateWork(50ms, &probe)`. `probe.max == 1`, 2 solutions.
  - `ExclusiveResources.DistinctResourcesOverlap`: resources `&r1` and `&r2` ⇒ `probe.max >= 2`.
  - `ExclusiveResources.ClearedOnReinit`: `fwd->resource = &r; t.init();` ⇒ `fwd->pimpl()->exclusiveResources()` contains `&r`. Then `fwd->resource = nullptr; t.reset(); t.init();` ⇒ empty.
  - `PlannerResources.DefaultSerializesInstance`: `StubPlanner p;` ⇒ `ElementsAre(&p)`.
  - `PlannerResources.StatelessBuiltinsAreShareable`: `JointInterpolationPlanner{}.exclusiveResources()` and `CartesianPath{}.exclusiveResources()` are empty.
  - `PlannerResources.MultiPlannerCollectsChildren`: `MultiPlanner{stub1, JointInterpolationPlanner, stub2}` ⇒ `UnorderedElementsAre(stub1.get(), stub2.get())`.
  - `PlannerResources.PipelinePlannersSharingAPipelineShareTheResource`: `int dummy;` then build a non-owning pipeline pointer: `planning_pipeline::PlanningPipelinePtr pipeline(std::shared_ptr<void>(), reinterpret_cast<planning_pipeline::PlanningPipeline*>(&dummy));` (aliasing constructor; it is never dereferenced). Then `solvers::PipelinePlanner a(pipeline), b(pipeline);` ⇒ `a.exclusiveResources()` and `b.exclusiveResources()` both equal `{pipeline.get()}`. Also `solvers::PipelinePlanner c(rclcpp::Node::SharedPtr());` before `init()` ⇒ `{&c}`.
  - `PlannerResources.MoveToAndConnectRegisterSharedPlanner`: one `auto stub = std::make_shared<StubPlanner>()` is shared by the task GEN{0} → `MoveTo("move", stub)` → `MoveRelative("rel", stub)` → `Connect("con", {{"group", stub}})` → GEN{0}, on `getModel()`, which has the group `"group"`. After `t.init()`, each of the three stages' `pimpl()->exclusiveResources()` equals `{stub.get()}`.
- [ ] **Step 2: Run `test-parallel`.** Expected: compile failure (`addExclusiveResource`, `exclusiveResources` unknown).
- [ ] **Step 3: Implement the API above.**
  - In `MoveTo::init` and `MoveRelative::init`, after `planner_->init(robot_model)`, call `addExclusiveResource` for each element of `planner_->exclusiveResources()`. In `Connect::init`, do the same for each valid planner in the loop.
  - The resources must be read *after* `planner_->init()`, because Humble's `PipelinePlanner` only gets its cached pipeline there.
  - Document the separate-pipeline recipe (§ Design) in the `PipelinePlanner` class comment.
- [ ] **Step 4: Run `test-parallel|test-move-to|test-move-relative`.** Expected: all pass.
- [ ] **Step 5: Commit:** `git commit -m "Serialize jobs sharing non-thread-safe planners via exclusive resources"`

---

### Task 6: Thread-safe random pose sampling

**Files:**
- Modify: `core/include/moveit/task_constructor/stages/generate_random_pose.h` (make `getPoseDimensionSampler` `protected`; add member `std::mt19937 engine_{ std::random_device{}() };`), `core/src/stages/generate_random_pose.cpp` (remove the namespace-scope `RANDOM_DEVICE`/`ENGINE`)
- Test: `core/test/test_generate_random_pose.cpp` (new, `mtc_add_gtest(test_generate_random_pose.cpp)`)

**Interfaces:**
- Consumes: nothing.
- Produces: no new public API; `getPoseDimensionSampler` becomes protected.

- [ ] **Step 1: Write the failing test.** `struct Probe : stages::GenerateRandomPose { using GenerateRandomPose::getPoseDimensionSampler; };` Then `TEST(GenerateRandomPose, SamplersKeepTheirOwnParameters)`: `a = p.getPoseDimensionSampler<std::uniform_real_distribution>(0.1)` and `b = …(0.1)`. Over 100 iterations, `EXPECT_NEAR(a(0.0), 0.0, 0.05)` and `EXPECT_NEAR(b(10.0), 10.0, 0.05)`. Today this fails: the function-`static` distribution keeps the first call's mean. It is also the shared state that makes concurrent sampling unsafe.
- [ ] **Step 2: Run `test-generate-random-pose`.** Expected: FAIL, because `b(10.0)` samples near 0.
- [ ] **Step 3: Implement.** Both specializations return `[this, width](double mean) { return Dist<double>(…mean…, …width…)(engine_); }`, building the distribution per call from `mean`. Keep the existing parameter mapping: normal → `(mean, stddev)`, uniform → `(mean − range/2, mean + range/2)`.
- [ ] **Step 4: Run `test-generate-random-pose`.** Expected: PASS.
- [ ] **Step 5: Commit:** `git commit -m "GenerateRandomPose: per-instance RNG, fix static distribution reuse"`

---

### Task 7: Python bindings

**Files:**
- Modify: `core/python/bindings/src/core.cpp`
- Test: `core/python/test/test_mtc.py` (runs in CI), `core/python/test/rostest_trampoline.py` (manual; its launch test is commented out in `core/python/CMakeLists.txt`)
- Note: Humble builds the bindings against the pybind11 `smart_holder` submodule. Every API used below (`def_property`, `py::call_guard<py::gil_scoped_release>`) exists there.

**Interfaces:**
- Consumes: `Task::setNumThreads/numThreads` (Task 3), `Stage::setParallelCompute/parallelCompute` (Task 2).
- Produces: Python `Task.num_threads` (int property) and `Stage.parallel_compute` (bool property). `Task.plan` releases the GIL.

- [ ] **Step 1: Write the failing tests.**
  - In `test_mtc.py`, `class TestTask(unittest.TestCase)`, `test_num_threads_property`: `t = core.Task(); assert t.num_threads == 1; t.num_threads = 0; assert t.num_threads == 0; t.num_threads = 3; assert t.num_threads == 3`. Also `s = stages.CurrentState(); assert s.parallel_compute; s.parallel_compute = False; assert not s.parallel_compute`.
  - In `rostest_trampoline.py`, add `test_parallel_monitoring_generator` and `test_parallel_propagator`. They copy `test_monitoring_generator` / `test_propagator` and set `task.num_threads = 4` before `self.plan(...)`, with the same `expected_solutions`. Without the GIL release they deadlock: the worker waits for the GIL that `plan()` holds.
- [ ] **Step 2: Run `test_mtc`.** Expected: FAIL, `AttributeError: 'Task' object has no attribute 'num_threads'`.
- [ ] **Step 3: Implement the bindings.**
  - On `Task`: `.def_property("num_threads", &Task::numThreads, &Task::setNumThreads, "int: worker threads used by plan(); 1: sequential, 0: hardware concurrency")`.
  - On `Stage`: `.def_property("parallel_compute", &Stage::parallelCompute, &Stage::setParallelCompute, "bool: allow compute() on a worker thread")`.
  - Add `py::call_guard<py::gil_scoped_release>()` to the existing `.def("plan", …)`. Python overrides re-acquire the GIL through `PYBIND11_OVERRIDE*`, and Python callables through pybind11's functional wrapper.
- [ ] **Step 4: Run `test_mtc`.** Expected: PASS. Then run the trampoline test manually with the launch file named in the commented `add_launch_test(... rostest_trampoline ...)` line of `core/python/CMakeLists.txt`. Expected: all tests pass, including `test_parallel_*`, and none hangs.
- [ ] **Step 5: Commit:** `git commit -m "Python: expose num_threads/parallel_compute, release GIL in Task.plan"`

---

### Task 8: Documentation and ThreadSanitizer CI

**Files:**
- Modify: `core/doc/howto.rst` (new section "Parallel planning")
- Modify: `.github/workflows/ci.yaml`
- Create: `.github/workflows/tsan.suppressions`

**Interfaces:**
- Consumes: the APIs of Tasks 2–7.
- Produces: user documentation and the `tsan` CI job.

- [ ] **Step 1: Write the "Parallel planning" section.** Cover, in this order:
  - enabling it (`task.setNumThreads(n)` / `task.num_threads = n`, the meaning of 1 and 0);
  - what runs in parallel and what doesn't (§ Design table, condensed);
  - the worker rules for stage authors (§ Design, verbatim);
  - planners: sharing one instance serializes it. On Humble, `PipelinePlanner`s with the same pipeline name share one cached pipeline and are serialized too; give each stage its own `planning_pipeline::PlanningPipeline` via `PipelinePlanner(pipeline)` (§ Design recipe). Use `addExclusiveResource` for custom shared objects;
  - escape hatch `setParallelCompute(false)`;
  - semantics: non-deterministic order, overrun bounds for timeout and `max_solutions`, callbacks always on the calling thread;
  - known limits (§ Design).
- [ ] **Step 2: Add the `tsan` CI job.** The branch's matrix currently has the single entry `- IMAGE: humble-source`. Add a second entry; the existing `UNDERLAY` logic picks up `/root/ws_moveit/install`:

  ```yaml
  - IMAGE: humble-source
    NAME: tsan
    DOCKER_RUN_OPTS: -e TSAN_OPTIONS="suppressions=$PWD/.github/workflows/tsan.suppressions second_deadlock_stack=1"
    TARGET_CMAKE_ARGS: -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-fsanitize=thread -O1 -g"
  ```

  Mark it non-blocking with `continue-on-error: ${{ matrix.env.NAME == 'tsan' }}` at job level, because the uninstrumented ROS and MoveIt binaries produce third-party noise.
- [ ] **Step 3: Write `tsan.suppressions`.** Seed it with `race:librcl*`, `race:librmw*`, `race:libfastrtps*`, `race:librclcpp*`. Never suppress a frame in `moveit::task_constructor`.
- [ ] **Step 4: Verify.** Push and open the CI run. Expected: all existing jobs green. In the `tsan` job log, `test-parallel` and `test-job-scheduler` report no `WARNING: ThreadSanitizer` whose top MTC frame is in `moveit::task_constructor`. If one does, it is a bug: fix it in the owning task's code. Do not suppress it.
- [ ] **Step 5: Commit:** `git commit -m "Document parallel planning, add ThreadSanitizer CI job"`

---

## Out of Scope / Follow-ups

0. **Jazzy port (next).** Notes for that plan, all checked against the code:
   - **Base.** Upstream has no `jazzy` branch. Jazzy was built by `ros2` up to tag `ros2-0.1.8` (`7c670ff`); later `ros2` commits require pybind11 v3, which Jazzy lacks (commit `dbf8955`).
   - **Same core code.** That tag differs from `humble` in the files this plan touches only by `.hpp` header spelling and unrelated fixes, so Tasks 1–4 and 6–8 cherry-pick almost unchanged.
   - **`PipelinePlanner`.** Jazzy's version creates its own pipelines per instance (no static cache) and has a per-instance `last_successful_planner_`. So it keeps the default `{this}`: drop the Humble override and its test.
   - **`LimitSolutions` test.** Jazzy has `LimitSolutions`; add `ParallelWrappers.LimitSolutions`: GEN×5 → `LimitSolutions(max 2)` around a slow FWD ⇒ exactly 2 solutions.
   - **CI.** CI image is `jazzy-release`.
   - **C++17** there as well.
1. Running container-internal work on workers (`ComputeIK`'s IK, `Merger`'s merging) through an explicit `prepare/work/commit` API for containers.
2. More than one in-flight job per stage. This needs per-job property snapshots (`performInitFrom(INTERFACE)` currently mutates the stage) and stage-local state isolation (`Connect::subsolutions_`, `upstream_solutions_`).
3. Letting stages share one `PlanningPipeline` concurrently. This needs a per-pipeline opt-in once a plugin is known to be reentrant. OMPL's per-request planning-context cache suggests it is, but that is not verified for every Humble plugin.
4. Making introspection's `get_solution` service safe against concurrent planning (pre-existing race).
5. A deterministic parallel mode (commit in submission order) for reproducible benchmarks.
