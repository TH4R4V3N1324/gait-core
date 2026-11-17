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

# Each test is <folder>/<name>.cpp, linked against that folder's library
set(GAIT_TESTS
  gait/test_body_pose
  gait/test_foot_directions
  gait/test_foot_force
  gait/test_gait_engine
  gait/test_gait_schedule
  gait/test_hexapod_leg_3dof
  gait/test_kinematics
  gait/test_leg_clearance
  gait/test_leg_odometry
)
set(BEHAVIOUR_TESTS
  behaviour/test_battery_monitor
)

function(gait_core_add_test path library)
  get_filename_component(name ${path} NAME)
  add_executable(${name} ${path}.cpp)
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})  # test_common.hpp
  target_link_libraries(${name} PRIVATE ${library} GTest::gtest_main)
  gtest_discover_tests(${name} DISCOVERY_TIMEOUT 30)
endfunction()

foreach(path ${GAIT_TESTS})
  gait_core_add_test(${path} gait_core)
endforeach()
foreach(path ${BEHAVIOUR_TESTS})
  gait_core_add_test(${path} gait_core_behaviour)
endforeach()

# Runs the gait and the behaviours together, counting every heap allocation
gait_core_add_test(test_no_heap gait_core_behaviour)

# Protocol: sizes and values from C++, plus a C translation unit, since controllers include it from
# C. Linked against gait_core too, to check Gait and Mode against the engine's values.
enable_language(C)
gait_core_add_test(protocol/test_packets gait_core)
target_sources(test_packets PRIVATE protocol/packets_c.c)
target_link_libraries(test_packets PRIVATE gait_core_protocol)

gait_core_add_test(teleop/test_gamepad_teleop gait_core_teleop)
target_link_libraries(test_gamepad_teleop PRIVATE gait_core_protocol)
