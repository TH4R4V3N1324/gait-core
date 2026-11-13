find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

include(GoogleTest)

set(GAIT_CORE_TESTS
  test_body_pose
  test_foot_directions
  test_foot_force
  test_gait_engine
  test_kinematics
  test_leg_clearance
  test_leg_odometry
)

foreach(name ${GAIT_CORE_TESTS})
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE gait_core GTest::gtest_main)
  gtest_discover_tests(${name} DISCOVERY_TIMEOUT 30)
endforeach()
