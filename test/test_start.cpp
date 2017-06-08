
#include <gtest/gtest.h>

//Run with "catkin_make run_tests_plane_calibration"
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
