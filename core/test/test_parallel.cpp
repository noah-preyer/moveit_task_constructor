#include <moveit/task_constructor/job_scheduler_p.h>
#include <moveit/task_constructor/task_p.h>

#include "stage_mockups.h"
#include "models.h"

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace moveit::task_constructor;
using testing::HasSubstr;

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
