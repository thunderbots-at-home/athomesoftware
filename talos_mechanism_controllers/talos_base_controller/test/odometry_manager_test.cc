#include <gtest/gtest.h>
#include <iostream>
#include "odometry_manager.h"

class OdometryManagerTest : public ::testing::Test 
{
  protected:
    OdometryManager odom_manager;
};

TEST_F (OdometryManagerTest, TestDefaultConstructor) {
  EXPECT_EQ (odom_manager.x_global(), 0.0);
  EXPECT_EQ (odom_manager.y_global(), 0.0);
  EXPECT_EQ (odom_manager.th_global(), 0.0);
  EXPECT_EQ (odom_manager.previous_timestamp(), 0.0);
}

TEST_F (OdometryManagerTest, TestGetCurrentTransform) {
  EXPECT_TRUE (false);
}

TEST_F (OdometryManagerTest, TestGetCurrentOdom) {
  EXPECT_TRUE (false);
}

TEST_F (OdometryManagerTest, TestToSecond){
  unsigned long t1 = 0.0;
  unsigned long t2 = 167;
  const double ToSec = odom_manager.ToSeconds(t2 - t1);
  EXPECT_EQ (odom_manager.ToSeconds(t2 - t1), (t2-t1)*(1.0/1000));
}

TEST_F (OdometryManagerTest, TestForward) {
  ArduinoMessage message;
  message.time_stamp = 167;
  message.l_feedback = 300;
  message.r_feedback = 300;

  

  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();


  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());

  const double expected_x = expected_x_delta + odom_manager.x_global();
  const double expected_y = expected_y_delta + odom_manager.y_global(); 
  const double expected_th = expected_theta_delta + odom_manager.th_global();

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp());

  odom_manager.UpdateOdometry(message);

  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel );
}

TEST_F (OdometryManagerTest, TestForwardLeft) {

  ArduinoMessage message;
  message.time_stamp = 200;
  message.l_feedback = 300;
  message.r_feedback = 400;

  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());

  const double expected_x = expected_x_delta + odom_manager.x_global();
  const double expected_y = expected_y_delta + odom_manager.y_global(); 
  const double expected_th = expected_theta_delta + odom_manager.th_global();

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp());

  odom_manager.UpdateOdometry(message);

  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel );
}

TEST_F (OdometryManagerTest, TestRotateRightLeft) {

  ArduinoMessage message;
  message.time_stamp = 180;
  message.l_feedback = 300;
  message.r_feedback = -300;
  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());

  const double expected_x = expected_x_delta + odom_manager.x_global();
  const double expected_y = expected_y_delta + odom_manager.y_global(); 
  const double expected_th = expected_theta_delta + odom_manager.th_global();

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp());

  odom_manager.UpdateOdometry(message);
  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel );

  ArduinoMessage message2;
  message2.time_stamp = 344;
  message2.l_feedback = -120;
  message2.r_feedback = 120;





  const double expected_left_delta2 = odom_manager.meters_per_tick() * message2.l_feedback;
  const double expected_right_delta2 = odom_manager.meters_per_tick() * message2.r_feedback;
  const double expected_distance2 = ( expected_left_delta2 + expected_right_delta2 ) / 2;
  const double expected_theta_delta2 = ( expected_left_delta2  - expected_right_delta2 ) / odom_manager.wheel_width();

  const double expected_x2_delta = expected_distance2 * cosf(expected_theta_delta2 + odom_manager.th_global());
  const double expected_y2_delta = expected_distance2 * sinf(expected_theta_delta2 + odom_manager.th_global());

  const double expected_x2 = expected_x2_delta + odom_manager.x_global();
  const double expected_y2 = expected_y2_delta + odom_manager.y_global();
  const double expected_th2 = expected_theta_delta2 + odom_manager.th_global();

  // ToSeconds(delta_t)
  const double expected_x_vel2 = expected_x2_delta / odom_manager.ToSeconds(message2.time_stamp - odom_manager.previous_timestamp()); 
  const double expected_y_vel2 = expected_y2_delta / odom_manager.ToSeconds(message2.time_stamp - odom_manager.previous_timestamp());
  const double expected_th_vel2 = expected_theta_delta2 / odom_manager.ToSeconds(message2.time_stamp - odom_manager.previous_timestamp());

  odom_manager.UpdateOdometry(message2);

  EXPECT_EQ ( odom_manager.x_global(), expected_x2 );
  EXPECT_EQ ( odom_manager.y_global(), expected_y2 );
  EXPECT_EQ ( odom_manager.th_global(), expected_th2 );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel2 );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel2 );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel2 );
}


TEST_F (OdometryManagerTest, TestFromNonZeroStart) {

  ArduinoMessage message;
  message.time_stamp = 133;
  message.r_feedback = +70;
  message.l_feedback = +121;
  
  odom_manager.x_global(5.4);
  odom_manager.y_global(-3.2);
  odom_manager.th_global(-2*M_PI+.01) ;

  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());


  const double expected_x = expected_x_delta + odom_manager.x_global();
  const double expected_y = expected_y_delta + odom_manager.y_global();
  const double expected_th = fmod((expected_theta_delta + odom_manager.th_global()),(2*M_PI));

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp());

  odom_manager.UpdateOdometry(message);

  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel ); 
}
TEST_F (OdometryManagerTest, TestBackRight) {
  ArduinoMessage message;
  message.time_stamp = 133;
  message.l_feedback = -150;
  message.r_feedback = -89;

  const double expected_left_delta = odom_manager.meters_per_tick() * message.l_feedback;
  const double expected_right_delta = odom_manager.meters_per_tick() * message.r_feedback;
  const double expected_distance = ( expected_left_delta + expected_right_delta ) / 2;
  const double expected_theta_delta = ( expected_left_delta  - expected_right_delta ) / odom_manager.wheel_width();

  const double expected_x_delta = expected_distance * cosf(expected_theta_delta + odom_manager.th_global());
  const double expected_y_delta = expected_distance * sinf(expected_theta_delta + odom_manager.th_global());


  const double expected_x = expected_x_delta + odom_manager.x_global();
  const double expected_y = expected_y_delta + odom_manager.y_global();
  const double expected_th = expected_theta_delta + odom_manager.th_global();

  // ToSeconds(delta_t)
  const double expected_x_vel = expected_x_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp()); 
  const double expected_y_vel = expected_y_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp());
  const double expected_th_vel = expected_theta_delta / odom_manager.ToSeconds(message.time_stamp - odom_manager.previous_timestamp());

  odom_manager.UpdateOdometry(message);

  EXPECT_EQ ( odom_manager.x_global(), expected_x );
  EXPECT_EQ ( odom_manager.y_global(), expected_y );
  EXPECT_EQ ( odom_manager.th_global(), expected_th );
  EXPECT_EQ ( odom_manager.x_vel(), expected_x_vel );
  EXPECT_EQ ( odom_manager.y_vel(), expected_y_vel );
  EXPECT_EQ ( odom_manager.th_vel(), expected_th_vel ); 
}

int main (int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
