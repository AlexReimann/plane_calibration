##############################################################################
# Tests
##############################################################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

catkin_add_gtest(${PROJECT_NAME}-test
  test_msg_eigen_converter.cpp
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

