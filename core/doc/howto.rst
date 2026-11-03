.. _sec-howtoguides:

How-To Guides
=============

.. _subsec-howto-stage-usage:

Stage Usage
-----------

.. _subsubsec-howto-alternatives:

Alternatives
^^^^^^^^^^^^

Using the ``alternatives`` stage, you can plan for multiple
execution paths.
Download the full example code here: :download:`Source <./../../demo/scripts/alternatives.py>`

.. literalinclude:: ./../../demo/scripts/alternatives.py
    :language: python
    :start-after: [initAndConfigAlternatives]
    :end-before: [initAndConfigAlternatives]

.. _subsubsec-howto-fallbacks:

Fallbacks
^^^^^^^^^

The ``fallbacks`` stage provides alternative motion planners
if planning fails with the primary one.
Download the full example code here: :download:`Source <./../../demo/scripts/fallbacks.py>`

.. literalinclude:: ./../../demo/scripts/fallbacks.py
    :language: python
    :start-after: [initAndConfigFallbacks]
    :end-before: [initAndConfigFallbacks]

.. _subsubsec-howto-merger:

Merger
^^^^^^

Plan and execute sequences in parallel using the ``merger`` stage.
Download the full example code here: :download:`Source <./../../demo/scripts/merger.py>`

.. literalinclude:: ./../../demo/scripts/merger.py
    :language: python
    :start-after: [initAndConfigMerger]
    :end-before: [initAndConfigMerger]

.. _subsubsec-howto-connect:

Connect
^^^^^^^

Connect two stages by finding a motion plan between them.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigConnect]
    :end-before: [initAndConfigConnect]

.. _subsubsec-howto-fix-collision-objects:

FixCollisionObjects
^^^^^^^^^^^^^^^^^^^

Check for collisions and resolve them if applicable.
Download the full example code here: :download:`Source <./../../demo/scripts/fix_collision_objects.py>`

.. literalinclude:: ./../../demo/scripts/fix_collision_objects.py
    :language: python
    :start-after: [initAndConfig]
    :end-before: [initAndConfig]

.. _subsubsec-howto-generate-place-pose:

GeneratePlacePose
^^^^^^^^^^^^^^^^^^^

Sample feasible poses around an object pose.
Considers geometry of primitive object type.
Solutions can be used for inverse
kinematics calculations.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initCollisionObject]
    :end-before: [initCollisionObject]

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigGeneratePlacePose]
    :end-before: [initAndConfigGeneratePlacePose]

.. _subsubsec-howto-generate-grasp-pose:

GenerateGraspPose
^^^^^^^^^^^^^^^^^

Sample poses around an object pose by providing
sample density ``angle_delta``.
Solutions can be used for inverse kinematics
calculations.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigGenerateGraspPose]
    :end-before: [initAndConfigGenerateGraspPose]

.. _subsubsec-howto-generate-pose:

GeneratePose
^^^^^^^^^^^^

Spawn a pose on new solutions of the monitored stage.
Download the full example code here: :download:`Source <./../../demo/scripts/generate_pose.py>`

.. literalinclude:: ./../../demo/scripts/generate_pose.py
    :language: python
    :start-after: [initAndConfigGeneratePose]
    :end-before: [initAndConfigGeneratePose]

.. _subsubsec-howto-pick:

Pick
^^^^

Wraps the task pipeline to execute a pick.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigPick]
    :end-before: [initAndConfigPick]

.. _subsubsec-howto-place:

Place
^^^^^

Wraps the task pipeline to execute a pick.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigPlace]
    :end-before: [initAndConfigPlace]

.. _subsubsec-howto-simplegrasp:

SimpleGrasp
^^^^^^^^^^^

Wraps the pose generation and inverse kinematics
computation for the pick pipeline.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigSimpleGrasp]
    :end-before: [initAndConfigSimpleGrasp]

.. _subsubsec-howto-simpleungrasp:

SimpleUnGrasp
^^^^^^^^^^^^^

Wraps the pose generation and inverse kinematics
computation for the place pipeline.
The code snippet is part of the :ref:`Pick and Place <subsec-tut-pick-place>` guide.
Download the full example code here: :download:`Source <./../../demo/scripts/pickplace.py>`

.. literalinclude:: ./../../demo/scripts/pickplace.py
    :language: python
    :start-after: [initAndConfigSimpleUnGrasp]
    :end-before: [initAndConfigSimpleUnGrasp]

.. _subsubsec-howto-modify-planning-scene:

ModifyPlanningScene
^^^^^^^^^^^^^^^^^^^

Modify the planning scene.
Download the full example code here: :download:`Source <./../../demo/scripts/modify_planning_scene.py>`

.. literalinclude:: ./../../demo/scripts/modify_planning_scene.py
    :language: python
    :start-after: [initAndConfigModifyPlanningScene]
    :end-before: [initAndConfigModifyPlanningScene]

.. _subsubsec-howto-fixed-state:

FixedState
^^^^^^^^^^

Spawn a pre-defined state.
Download the full example code here: :download:`Source <./../../demo/scripts/fixed_state.py>`

.. literalinclude:: ./../../demo/scripts/fixed_state.py
    :language: python
    :start-after: [initAndConfigFixedState]
    :end-before: [initAndConfigFixedState]

.. _subsubsec-howto-compute-ik:

ComputeIK
^^^^^^^^^

Compute the inverse kinematics of the monitored stages'
solution. Be sure to correctly configure the ``target_pose``
property to be derived from the monitored stage as shown
in the example.
Download the full example code here: :download:`Source <./../../demo/scripts/compute_ik.py>`

.. literalinclude:: ./../../demo/scripts/compute_ik.py
    :language: python
    :start-after: [initAndConfigComputeIk]
    :end-before: [initAndConfigComputeIk]

.. _subsubsec-howto-move-to:

MoveTo
^^^^^^

Use planners to compute a motion plan.
Download the full example code here: :download:`Source <../../demo/scripts/cartesian.py>`

.. literalinclude:: ../../demo/scripts/cartesian.py
    :language: python
    :start-after: [initAndConfigMoveTo]
    :end-before: [initAndConfigMoveTo]

.. _subsubsec-howto-move-relative:

MoveRelative
^^^^^^^^^^^^

Move along a relative offset.
Download the full example code here: :download:`Source <../../demo/scripts/cartesian.py>`

.. literalinclude:: ../../demo/scripts/cartesian.py
    :language: python
    :start-after: [initAndConfigMoveRelative]
    :end-before: [initAndConfigMoveRelative]

.. _subsec-howto-stage-extension:

Stage Extension
---------------

.. _subsubsec-howto-move-relative-extension:

MoveRelative
^^^^^^^^^^^^

You may derive from this stage to extend its functionality.
``MoveRelative`` itself derives from the propagator stage that
alters solutions (i.e. computes a motion plan) when they are
passed through the stage.

.. literalinclude:: ./../../core/python/test/rostest_trampoline.py
    :language: python
    :pyobject: PyMoveRelX

.. _subsubsec-howto-generator-extension:

Generator
^^^^^^^^^

Derive from the ``Generator`` stage to implement your own
logic in the compute function.

.. literalinclude:: ./../../core/python/test/rostest_trampoline.py
    :language: python
    :pyobject: PyGenerator

.. _subsubsec-howto-monitoring-generator-extension:

MonitoringGenerator
^^^^^^^^^^^^^^^^^^^

Derive from the ``MonitoringGenerator`` stage to
implement your own logic in the compute function.
Use the monitoring generator instead of the normal
generator if you need to access solutions of the
monitored stage (e.g. computation of inverse kinematics).

.. literalinclude:: ./../../core/python/test/rostest_trampoline.py
    :language: python
    :pyobject: PyMonitoringGenerator

.. _subsec-howto-parallel-planning:

Parallel planning
-----------------

By default, ``Task::plan()`` computes all stages one after the other in the thread that calls it.
Stages that spend most of their time in motion planning (``MoveTo``, ``MoveRelative``, ``Connect``) can instead
run concurrently on a pool of worker threads, which speeds up tasks with many alternative solutions.

.. _subsubsec-howto-parallel-enable:

Enabling parallel planning
^^^^^^^^^^^^^^^^^^^^^^^^^^

Set the number of threads on the task, before calling ``plan()``:

.. code-block:: cpp

    task.setNumThreads(4);  // 4 worker threads
    task.plan();

.. code-block:: python

    task.num_threads = 4  # 4 worker threads
    task.plan()

The value has the following meaning:

``1`` (default)
    Sequential planning. All stages are computed in the thread calling ``plan()``, exactly as without this feature.

``0``
    Use as many worker threads as ``std::thread::hardware_concurrency()`` reports.

``n >= 2``
    Use ``n`` worker threads.

.. _subsubsec-howto-parallel-what-runs-where:

What runs in parallel
^^^^^^^^^^^^^^^^^^^^^

The thread calling ``plan()`` (the *planning thread*) owns the solution graph. Everything that creates, changes or
removes solutions and states happens there. Only the ``compute()`` of the leaf stages listed on the right runs on
worker threads:

.. list-table::
    :header-rows: 1
    :widths: 50 50

    * - Planning thread (owner of the graph)
      - Worker threads
    * - ``Task::plan`` loop and ``canCompute()`` of all stages (including a user's ``Generator::canCompute``)
      - ``Generator::compute()``
    * - Container ``compute()`` logic (``Serial``, ``Alternatives``, ``Fallbacks``, ``Merger``, wrappers including
        the IK of ``ComputeIK``)
      - ``PropagatingEitherWay::computeForward()`` / ``computeBackward()`` (hence ``MoveTo``, ``MoveRelative``,
        ``ModifyPlanningScene``, ...)
    * - Job preparation: taking the input state (or pair of states) off the interface
      - ``Connecting::compute(from, to)`` (hence ``Connect``)
    * - Replaying what a worker recorded: ``spawn``, ``send*``, ``connect``, ``silentFailure`` (storing solutions,
        updating interfaces, lifting solutions to the parent containers, pruning)
      - Cost term evaluation for the solutions created by the job
    * - ``MonitoringGenerator::onNewSolution``, ``Connecting::compatible``, solution and task callbacks,
        introspection
      - Planner calls (``PlannerInterface::plan``)

Per stage, at most one job is in flight at any time. Concurrency therefore comes from different stages working on
different states at the same time, not from one stage working on several states at once.

.. _subsubsec-howto-parallel-worker-rules:

Rules for stage authors
^^^^^^^^^^^^^^^^^^^^^^^

The worker side of a stage (``compute()`` of a ``Generator``, ``computeForward()`` / ``computeBackward()`` of a
``PropagatingEitherWay`` and ``compute(from, to)`` of a ``Connecting`` stage) may:

- read its own stage's members and properties;
- read ``InterfaceState::scene()`` and ``InterfaceState::properties()`` of its inputs;
- call ``spawn``, ``sendForward``, ``sendBackward``, ``connect``, ``silentFailure``, ``storeFailures``, and planners.

It must not read ``InterfaceState::priority()``, the incoming or outgoing trajectories of a state, other stages, or
task state.

``Connecting::compatible()`` may run on the planning thread while the ``compute()`` of the same stage runs on a
worker. Do not share mutable state between the two without synchronization.

Python stage overrides (and Python cost terms) also run on worker threads. They re-acquire the GIL, so the Python
code of different stages runs one thread at a time.

.. _subsubsec-howto-parallel-planners:

Planners and exclusive resources
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Planners are assumed not to be thread-safe. Stages that share one planner instance never plan at the same time:
their jobs are serialized, so sharing a planner between stages gives no speed-up. Give every stage that should plan
concurrently its own planner instance.

``JointInterpolationPlanner`` and ``CartesianPath`` are stateless and are never serialized. A ``MultiPlanner`` is
serialized exactly like the planners it wraps.

``MoveTo``, ``MoveRelative`` and ``Connect`` register the resources of their planners automatically.

**PipelinePlanner on Humble.**
On Humble, ``PipelinePlanner::create()`` keeps a process-wide cache of planning pipelines, keyed by robot model,
pipeline namespace and adapter parameter. All ``PipelinePlanner`` instances created for the same pipeline name and
robot model (e.g. ``PipelinePlanner(node, "ompl")``) therefore share one ``planning_pipeline::PlanningPipeline``,
and their jobs are serialized even if each stage has its own ``PipelinePlanner`` instance: some planner plugins are
not reentrant.
Pipelines with different namespaces (e.g. ``ompl`` and ``pilz_industrial_motion_planner``) run in parallel.

To plan with the same planner plugin in parallel, create a separate pipeline for each stage and pass it to the
planner. Planners are given to a stage when it is constructed (``MoveTo`` and ``MoveRelative`` take the planner as
second constructor argument, ``Connect`` takes a ``GroupPlannerVector``):

.. code-block:: cpp

    #include <moveit/planning_pipeline/planning_pipeline.h>
    #include <moveit/task_constructor/solvers/pipeline_planner.h>
    #include <moveit/task_constructor/stages/move_to.h>

    namespace mtc = moveit::task_constructor;

    // every call creates a separate pipeline, hence a separate planner plugin instance
    mtc::solvers::PlannerInterfacePtr makeOmplPlanner(const rclcpp::Node::SharedPtr& node,
                                                      const moveit::core::RobotModelConstPtr& robot_model) {
        auto pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
            robot_model, node, "ompl", "planning_plugin", "request_adapters");
        return std::make_shared<mtc::solvers::PipelinePlanner>(pipeline);
    }

    // after task.loadRobotModel(node): the robot model must be the one of the task
    task.add(std::make_unique<mtc::stages::MoveTo>("move a", makeOmplPlanner(node, task.getRobotModel())));
    task.add(std::make_unique<mtc::stages::MoveTo>("move b", makeOmplPlanner(node, task.getRobotModel())));

Constructing a ``PipelinePlanner`` from an existing pipeline is available in C++ only.

**Custom shared objects.**
If your own stage uses an object that must not be used by two threads at once (a service client, a device handle, a
non-reentrant library), declare it with ``ComputeBase::addExclusiveResource()`` in ``init()``. Stages declaring the
same address never compute at the same time; workers pick the first queued job whose resources are all free:

.. code-block:: cpp

    class MyStage : public PropagatingForward {
    public:
        void init(const moveit::core::RobotModelConstPtr& robot_model) override {
            PropagatingForward::init(robot_model);  // forgets the resources of a previous init()
            addExclusiveResource(shared_client_.get());
        }
        // ...

    private:
        std::shared_ptr<Client> shared_client_;  // shared with other stages
    };

``init()`` clears the declared resources, so register them after calling the base class's ``init()``, every time.
A custom planner can declare its own resources by overriding ``PlannerInterface::exclusiveResources()``, which
returns ``{this}`` by default.

.. _subsubsec-howto-parallel-compute-flag:

Opting out per stage
^^^^^^^^^^^^^^^^^^^^

If the ``compute()`` of a stage is not safe to run on a worker, disable parallel computation for this stage:

.. code-block:: cpp

    stage->setParallelCompute(false);

.. code-block:: python

    stage.parallel_compute = False

The flag defaults to ``true`` and only matters for ``Generator``, ``Propagating*`` and ``Connecting`` stages while
the task uses more than one thread; containers ignore it. Such a stage is computed on the planning thread, like
without parallel planning. While it computes, jobs that are already running on workers continue, but no new job is
dispatched.

.. _subsubsec-howto-parallel-semantics:

Semantics
^^^^^^^^^

- **The order of solutions is not deterministic.** Which solution is found first depends on thread timing, so
  repeated runs of the same task can find the same solutions in a different order. Do not rely on the order, e.g.
  in tests.
- **Callbacks always run on the thread that called** ``plan()``. This holds for task callbacks, solution callbacks
  and the publishing of introspection data. They need no locking against the planning loop.
- **Timeout, preemption and max_solutions.** When planning ends for one of these reasons, no new job is started.
  Queued jobs are cancelled before they start, and their inputs are put back, so a later ``plan()`` continues where
  this one stopped, as with sequential planning. Jobs that are already running are waited for and their results are
  kept. The overrun is therefore bounded by the longest running job, and the number of solutions can exceed
  ``max_solutions`` by the solutions completed by the jobs still running when the limit was reached.
- **Exceptions.** An exception thrown by a stage on a worker is not lost: planning stops as described above, and
  ``plan()`` rethrows the exception once all running jobs have finished.
  An exception thrown on the planning thread, e.g. by a solution or task callback, aborts planning: ``plan()``
  waits for the running jobs, discards their results and rethrows the exception. The inputs of the discarded jobs
  are put back as well, so a later ``plan()`` computes them again. Generators take no input, so the result of a
  discarded generator job is lost.
- **Compute time.** In parallel mode, the compute time reported for a container is the sum of the job times of its
  children. It can be larger than the wall-clock time.
- **Python.** ``Task.plan()`` releases the GIL while planning, and Python stage overrides and Python cost terms
  re-acquire it. Other Python threads can therefore run while the task is planning. Do not modify the task, its
  stages or their properties from another thread during that time.

.. _subsubsec-howto-parallel-limits:

Known limits
^^^^^^^^^^^^

- **Container-internal work stays on the planning thread.** This covers the IK computation of ``ComputeIK`` and the
  trajectory merging of ``Merger``. Only the leaf stages listed above run on workers.
- **Kinematics plugins are assumed to be thread-safe.** MoveIt makes the same assumption for parallel OMPL attempts.
  If a plugin is not, disable parallel computation (``setParallelCompute(false)``) on the affected stages.
- **No backpressure.** A fast generator can run far ahead of a slow downstream stage, and the generated states pile
  up in between. A generator that never ends is therefore a problem in parallel mode. Make generators finite or
  bounded, e.g. keep the ``max_solutions`` property of ``GenerateRandomPose`` small.
- **Fallbacks.** While any job of the current child of a ``Fallbacks`` container (or of the child's descendants) is
  in flight, ``Fallbacks`` starts nothing new for that child and does not move on to the next child. Thus, the stages
  of a child consisting of several stages do not compute concurrently. (This applies to generator and propagator
  ``Fallbacks``. In a connecting ``Fallbacks``, a child can already work on a pair that failed in the previous child
  while that child computes its next pair.)
- **At most one job per stage.** Concurrency comes from different stages working at the same time. A single slow
  stage is not sped up.
- **Introspection.** The existing race between the ``get_solution`` service of the introspection and planning is
  unchanged.
- **Planning again after an early stop (known issue).** If ``plan()`` stopped early (``max_solutions``, timeout or
  preemption) while children of a container (e.g. ``Alternatives``) had queued jobs, calling ``plan()`` again
  without ``reset()`` can crash. The cause is a pre-existing bug: ``init()`` replaces the interfaces of containers.
  Call ``reset()`` before planning again in this situation.
- **Planner sharing.** Stages that share a planner, or that use ``PipelinePlanner`` instances on the same cached
  pipeline, are serialized (see above).
