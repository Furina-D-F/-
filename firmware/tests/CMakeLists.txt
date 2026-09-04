cmake_minimum_required(VERSION 3.16)

project(robot_driver_tests C)

enable_testing()

set(UNITY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Unity/src)
set(DRIVERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../drivers)

add_executable(robot_driver_unity_tests
    ${UNITY_DIR}/unity.c
    unity_driver_test.c
    ${DRIVERS_DIR}/uart.c
    ${DRIVERS_DIR}/protocol.c
    ${DRIVERS_DIR}/joint_motor.c
)

target_compile_features(robot_driver_unity_tests PRIVATE c_std_11)
target_compile_options(robot_driver_unity_tests PRIVATE -Wall -Wextra -Werror)
target_include_directories(robot_driver_unity_tests PRIVATE
    ${UNITY_DIR}
    ${DRIVERS_DIR}
)
target_link_libraries(robot_driver_unity_tests PRIVATE m)

add_test(NAME robot_driver_unity_tests COMMAND robot_driver_unity_tests)
