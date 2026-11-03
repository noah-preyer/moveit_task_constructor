# Parallel planning (ROS 2 Humble)

This branch adds **parallel planning** to the MoveIt Task Constructor (MTC): `Task::plan()` can run the
`compute()` of independent stages on a pool of worker threads. Expensive motion planning (OMPL, Pilz, Cartesian
paths, ...) of different stages and alternatives then overlaps in time instead of running one after the other.

- **Opt-in.** The default (`numThreads() == 1`) is the sequential planning loop, unchanged.
- **Same results.** For deterministic stages, the tests check that parallel planning finds the same solutions as
  sequential planning. Only their order can differ. With pruning enabled, parallel planning may find additional
  valid solutions, because a job can finish on a state that was pruned in the meantime.
- **Target:** ROS 2 Humble, the `humble` branch of MTC.

The complete user documentation (rules for stage authors, planner setup, semantics, known limits) is the
[Parallel planning section of the how-to guides](core/doc/howto.rst#parallel-planning). This file summarizes the
change for reviewers and users.

## Contents

- [Quick start](#quick-start)
- [How it works](#how-it-works)
- [Getting a speed-up: planners](#getting-a-speed-up-planners)
- [API changes](#api-changes)
- [Behaviour changes outside parallel mode](#behaviour-changes-outside-parallel-mode)
- [Implementation overview](#implementation-overview)
- [Tests and CI](#tests-and-ci)
- [Known issues and limitations](#known-issues-and-limitations)
- [Follow-ups](#follow-ups)

## Quick start

C++:

```cpp
moveit::task_constructor::Task task;
// ... load the robot model, add stages ...
task.setNumThreads(4);  // 1 (default): sequential, 0: one thread per core, n >= 2: n worker threads
task.plan(5);
```

Python:

```python
task = core.Task()
# ... load the robot model, add stages ...
task.num_threads = 4
task.plan(max_solutions=5)
```

A stage whose `compute()` must not run on a worker thread can opt out:

```cpp
stage->setParallelCompute(false);  // Python: stage.parallel_compute = False
```

To actually plan in parallel, stages must not share one planner instance (see
[Getting a speed-up: planners](#getting-a-speed-up-planners)).

## How it works

**Compute in parallel, commit serially.** MTC's solution graph is not thread-safe: interfaces, `InterfaceState`
links, container bookkeeping, pruning, callbacks and introspection are tightly coupled, and locking all of them would
be slow and deadlock-prone. Only the expensive part runs on workers instead:

```text
planning thread (calls plan())                       worker threads
---------------------------------------------------  ----------------------------------------------
traverse stages; a computable leaf stage gets a job:
  prepare: take one input off its interface   ---->  run the stage's compute() (planners, IK, ...)
           (a state, or a pair for Connect)          spawn / sendForward / sendBackward / connect /
                                                     silentFailure are *recorded*, not applied
commit: replay the recorded calls             <----  job finished
  (store solutions, update interfaces, lift
   solutions to containers, prune, run callbacks)
```

- **Leaf stages run on workers:** `Generator`, `PropagatingEitherWay` (`MoveTo`, `MoveRelative`,
  `ModifyPlanningScene`, ...) and `Connecting` (`Connect`).
- **Containers stay on the planning thread:** their scheduling logic, and the internal work of `ComputeIK` (IK) and
  `Merger` (trajectory merging).
- **At most one job per stage** is in flight. While it runs, the stage is *busy*: it reports `canCompute() == false`,
  so containers skip it, and notifications for it (e.g. new upstream solutions of a `MonitoringGenerator`) are queued
  and delivered when the job finishes.
- **Exclusive resources.** A job lists objects it must use alone, e.g. a non-thread-safe planner. Workers take the
  first queued job whose resources are all free, so no stage starves another.
- **Callbacks** (solution callbacks, task callbacks, introspection) always run on the thread that called `plan()`.
- **Stopping.** On `max_solutions`, timeout or preempt, no new job starts. Queued jobs are cancelled and their inputs
  are put back into the graph. Running jobs are waited for and committed. A worker exception is rethrown by `plan()`
  after all running jobs have finished. `plan()` never returns while a worker still works on the task.

## Getting a speed-up: planners

Planners are assumed **not** to be thread-safe. Stages that share a planner instance never plan at the same time;
their jobs are serialized.

| Planner | Parallel use |
|---|---|
| `JointInterpolationPlanner`, `CartesianPath` | Stateless: never serialized, can be shared freely |
| `MultiPlanner` | Serialized exactly like the planners it wraps |
| `PipelinePlanner` (Humble) | Serialized per **planning pipeline** (see below) |
| Your own `PlannerInterface` | Serialized per instance, unless you override `exclusiveResources()` |

`MoveTo`, `MoveRelative` and `Connect` register their planners' resources automatically in `init()`.

**`PipelinePlanner` on Humble.** `PipelinePlanner::create()` keeps a process-wide cache of planning pipelines, keyed
by robot model, pipeline namespace and adapter parameter. Every `PipelinePlanner(node, "ompl")` therefore shares one
`planning_pipeline::PlanningPipeline`, and all stages using it are serialized. This applies even if each stage has
its own `PipelinePlanner` instance. Pipelines with different namespaces (e.g. `ompl` and
`pilz_industrial_motion_planner`) do run in parallel.

To run several OMPL plans at once, give each stage its own pipeline:

```cpp
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/move_to.h>

namespace mtc = moveit::task_constructor;

// every call creates a separate pipeline, hence a separate planner plugin instance
mtc::solvers::PlannerInterfacePtr makeOmplPlanner(const rclcpp::Node::SharedPtr& node,
                                                  const moveit::core::RobotModelConstPtr& robot_model) {
	auto pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "ompl",
	                                                                      "planning_plugin", "request_adapters");
	return std::make_shared<mtc::solvers::PipelinePlanner>(pipeline);
}

// after task.loadRobotModel(node): the pipeline must use the task's robot model
task.add(std::make_unique<mtc::stages::MoveTo>("move a", makeOmplPlanner(node, task.getRobotModel())));
task.add(std::make_unique<mtc::stages::MoveTo>("move b", makeOmplPlanner(node, task.getRobotModel())));
```

This recipe has been compile-checked but not yet run against a real MoveIt configuration.

A custom stage that uses its own shared object (a service client, a device handle) declares it with
`ComputeBase::addExclusiveResource(ptr)` in `init()`, after calling the base class's `init()`.

## API changes

All changes are source compatible: existing stages, containers and planners compile unchanged. **The ABI changes**:
new virtual functions and members in `Stage`, `ComputeBase`, `PlannerInterface`, `Interface`, `Task`,
`GenerateRandomPose` and the private stage classes. Downstream packages must be rebuilt.

| Where | New API | Notes |
|---|---|---|
| `Task` | `void setNumThreads(unsigned int)`, `unsigned int numThreads() const` | default 1 |
| `Stage` | `void setParallelCompute(bool)`, `bool parallelCompute() const` | default `true`; only affects compute stages |
| `ComputeBase` | `void init(...) override`; protected `void addExclusiveResource(const void*)` | `init()` clears the resources |
| `solvers::PlannerInterface` | `virtual std::vector<const void*> exclusiveResources() const` | default `{this}` |
| `JointInterpolationPlanner`, `CartesianPath` | `exclusiveResources()` returns `{}` | stateless |
| `MultiPlanner` | `exclusiveResources()` returns its children's resources | |
| `PipelinePlanner` (Humble) | `exclusiveResources()` returns its (cached) pipeline | `{this}` before `init()` |
| `Interface` | `void reinsert(container_type&& removed)` | puts back a state taken by `remove()`, keeps its priority, no notification |
| `stages::GenerateRandomPose` | `getPoseDimensionSampler()` is now `protected` | was `private` |
| Python `Task` | `num_threads` property; `plan()` releases the GIL | |
| Python `Stage` | `parallel_compute` property | |

## Behaviour changes outside parallel mode

- **`GenerateRandomPose`** now owns its random number engine, and every sampler uses its own mean and width.
  Previously all instances shared one global engine (a data race under threads). Function-`static` distributions also
  kept the mean and width of the *first* sampler of each distribution type for every later sampler. This is a bug
  fix, but it changes the sampled poses in sequential mode too.
- **Python `Task.plan()`** releases the GIL in every mode, so other Python threads can run while a task plans. Do not
  modify the task from another thread during `plan()`.
- **`core/src/properties.cpp`** gets the `<boost/core/demangle.hpp>` include, a backport of upstream `b8b53cf`. It is
  required to build against newer Boost.

Everything else is unchanged with `numThreads() == 1`. The existing test suites pass unmodified.

## Implementation overview

| File | Change |
|---|---|
| `core/include/moveit/task_constructor/job_scheduler_p.h`, `core/src/job_scheduler.cpp` | New, private: `Job` and `JobScheduler`. A generic worker pool with exclusive resources, FIFO order and a finished-job queue drained on the planning thread. `drain()` cancels queued jobs and commits running ones; `abort()` cancels everything. Thread count 0 is an *inline* mode used by tests. |
| `core/include/moveit/task_constructor/stage_p.h`, `core/src/stage.cpp` | The job path: `runCompute()` submits a job for a computable compute stage; `prepareJob()` per leaf type; `commitOrDefer()` records spawn/send/connect/silentFailure on the worker; `onJobFinished()` / `onJobCancelled()` keep the in-flight counters of the stage and its ancestors, deliver queued notifications, replay commits and put back the inputs of cancelled jobs; `deferWhileBusy()` for `MonitoringGenerator`. |
| `core/include/moveit/task_constructor/task.h`, `task_p.h`, `core/src/task.cpp` | `setNumThreads()`; `plan()` dispatches to the unchanged sequential loop (`planSequential`) or to `planParallel` (dispatch, wait, commit; stop on preempt / timeout / `max_solutions` / error; drain or abort on every exit path). |
| `core/src/container.cpp` | `Fallbacks` waits for a busy current child instead of moving on, and passes a failed pair to the child after the failing one. |
| `core/include/moveit/task_constructor/storage.h`, `core/src/storage.cpp` | `Interface::reinsert()`. |
| `core/include/moveit/task_constructor/solvers/*`, `core/src/solvers/*` | `exclusiveResources()` and its overrides. |
| `core/src/stages/{move_to,move_relative,connect}.cpp` | Register the planners' resources in `init()`. |
| `core/src/stages/generate_random_pose.cpp`, `stages/generate_random_pose.h` | Per-instance RNG. |
| `core/python/bindings/src/core.cpp` | `num_threads`, `parallel_compute`, GIL release in `plan()`. |
| `core/doc/howto.rst` | "Parallel planning" section. |
| `.github/workflows/ci.yaml`, `.github/workflows/tsan.suppressions` | ThreadSanitizer CI job. |

The implementation plan, with the full design, is in
[`docs/superpowers/plans/2026-11-02-parallel-planning-humble.md`](docs/superpowers/plans/2026-11-02-parallel-planning-humble.md).

## Tests and CI

New and extended test suites in `moveit_task_constructor_core`:

| Test | What it covers |
|---|---|
| `test-job-scheduler` (22 tests) | Inline mode, deferred commits, concurrency, exclusive resources, FIFO with blocked jobs, errors, `drain()` / `abort()`, deadlines |
| `test-parallel` (32 tests) | `InlineJobs` (5): job path in inline mode. `ParallelPlanning` (11): concurrency, same results as sequential, threads of callbacks, preempt, timeout, `max_solutions`, worker exceptions, `MonitoringGenerator` while busy, compute time, `numThreads()`. `ParallelFallbacks` (4) and `ParallelWrappers` (1). `ExclusiveResources` (3) and `PlannerResources` (5). `ResumePlanning` (3): `plan()` after an early stop, and the abort path. |
| `test-generate-random-pose` | Samplers keep their own parameters |
| `test-interface-state` | `Interface::reinsert` keeps priority, sends no notification |
| `test-{container,serial,pruning,fallback}-inline` | The existing suites, rerun with every compute stage going through the job path (inline mode); the results must be identical |
| `test_mtc` (Python) | `num_threads` / `parallel_compute` properties |

Run them in a Humble workspace:

```bash
colcon build --packages-up-to moveit_task_constructor_core
colcon test --packages-select moveit_task_constructor_core --ctest-args -R "test-parallel|test-job-scheduler|inline"
colcon test-result --verbose
```

**CI** (`.github/workflows/ci.yaml`) runs two jobs on `moveit/moveit2:humble-source`:

- the regular build and tests;
- `humble-source • tsan`: a clang build with `-fsanitize=thread` (GCC 11's ThreadSanitizer misses an interceptor and
  reports false races). It is non-blocking, because uninstrumented ROS libraries produce third-party reports.
  `tsan.suppressions` covers those by library name. A ThreadSanitizer report with a frame in
  `moveit::task_constructor` is a bug and must never be suppressed.

## Known issues and limitations

**Open issue: planning again after an early stop can crash in parallel mode.**

- **Trigger:** parallel mode, a container (e.g. `Alternatives`) whose children have queued jobs (typically because
  they share a planner), `plan()` stopped by `max_solutions`, timeout or preempt, and then `plan()` called again
  without `reset()`.
- **Result:** that second `plan()` can segfault.
- **Cause:** a pre-existing MTC bug. Every `init()` replaces a container's interfaces and leaves dangling pointers
  behind. Sequential planning does not stop inside a container, so it does not hit this case.
- **Workaround:** call `task.reset()` before planning again after an early stop in parallel mode.

Other limitations (details in the [how-to guide](core/doc/howto.rst#parallel-planning)):

- **Containers' own work is not parallel.** `ComputeIK`'s IK and `Merger`'s merging run on the planning thread.
- **Kinematics plugins are assumed thread-safe**, as MoveIt assumes for parallel OMPL attempts. Use
  `setParallelCompute(false)` on stages using a plugin that is not.
- **No backpressure.** A fast, never-ending generator runs far ahead of a slow downstream stage, and states pile up.
  Keep generators finite.
- **Fallbacks:** while a child (or one of its descendants) has a job in flight, `Fallbacks` starts nothing new for it.
- **One job per stage:** a single slow stage is not sped up.
- **Order and counts:** solution order is not deterministic. `max_solutions` can be exceeded by the solutions of
  jobs still running when the limit was reached.
- **Not covered by CI:** the Python launch tests for parallel planning (`rostest_trampoline.py`, disabled in CMake).
  That Python stages do not deadlock was verified only with a separate script outside the repository.
- **Pre-existing, unrelated:** nesting `Fallbacks` inside `Alternatives` crashes in `init()`. The introspection
  `get_solution` service races with planning.

## Follow-ups

1. Fix the container interface lifetime bug behind the open issue. This also benefits sequential re-planning.
2. A CI-visible Python test for the GIL release (e.g. a pure-Python generator with `num_threads = 2` and a timeout).
3. Smoke-test the separate-pipeline recipe with a real MoveIt configuration.
4. Optional: backpressure for fast generators, parallel container-internal work (`ComputeIK`), several jobs per stage.
5. Port to Jazzy. Base it on tag `ros2-0.1.8`, the last `ros2` release built on Jazzy. Jazzy's `PipelinePlanner`
   creates its pipelines per instance, so the Humble `exclusiveResources()` override is not needed there.
