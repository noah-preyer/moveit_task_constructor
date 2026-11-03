#include <moveit/task_constructor/stages/generate_random_pose.h>

#include <random>

#include <gtest/gtest.h>

using namespace moveit::task_constructor;

namespace {

// Expose the protected sampler factory
struct Probe : stages::GenerateRandomPose
{
	using GenerateRandomPose::getPoseDimensionSampler;
};

}  // namespace

// Each sampler must use the parameters it was created with. A function-static distribution inside the sampler would
// stay initialized with the mean of the very first call.
TEST(GenerateRandomPose, SamplersKeepTheirOwnParameters) {
	Probe p;
	const auto a = p.getPoseDimensionSampler<std::uniform_real_distribution>(0.1);
	const auto b = p.getPoseDimensionSampler<std::uniform_real_distribution>(0.1);
	for (int i = 0; i < 100; ++i) {
		EXPECT_NEAR(a(0.0), 0.0, 0.05);
		EXPECT_NEAR(b(10.0), 10.0, 0.05);
	}
}
