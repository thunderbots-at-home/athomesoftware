#include <gtest/gtest.h>
#include <odometry_manager.h>

class OdometryManagerTest : public ::testing::Test 
{
  protected:
    OdometryManager odom_manager;
};

TEST_F (OdometryManagerTest, TestSetup) {
  EXPECT_EQ (odom_manager.x(), 0.0);
  EXPECT_EQ (odom_manager.y(), 0.0);
  EXPECT_EQ (odom_manager.th(), 0.0);
  EXPECT_EQ (odom_manager.last_time_stamp(), 0.0);
}

TEST_F (OdometryManagerTest, TestUpdateOdometry) {
  EXPECT_TRUE (false);
}

TEST_F (OdometryManagerTest, TestGetCurrentTransform) {
  EXPECT_TRUE (false);
}

TEST_F (OdometryManagerTest, TestGetCurrentOdom) {
  EXPECT_TRUE (false);
}

TEST_F (OdometryManagerTest, TestForward) {
  EXPECT_TRUE (false);
}

TEST_F (OdometryManagerTest, TestRotateRight) {
  EXPECT_TRUE (false);
}

TEST_F (OdometryManagerTest, TestBackRight) {
  EXPECT_TRUE (false);
}

int main (int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
