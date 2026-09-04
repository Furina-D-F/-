cmake_minimum_required(VERSION 3.16)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

project(robot_firmware C ASM)

set(MCU_FLAGS -mcpu=cortex-m4 -mthumb)

set(FREERTOS_DIR ${CMAKE_SOURCE_DIR}/third_party/FreeRTOS-Kernel)

add_executable(robot_firmware
    app/main.c
    app/control.c
    app/scheduler_validation.c
    bsp/startup.s
    bsp/clock.c
    bsp/gpio.c
    bsp/timer.c
    bsp/qemu_uart.c
    drivers/joint_motor.c
    drivers/protocol.c
    drivers/uart.c
    drivers/communication.c
    lib/memory.c
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

target_compile_options(robot_firmware PRIVATE
    ${MCU_FLAGS}
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffreestanding
    -fdata-sections
    -ffunction-sections
    -Wall
    -Wextra
)

target_compile_definitions(robot_firmware PRIVATE ROBOT_QEMU=1)

target_include_directories(robot_firmware PRIVATE
    ${CMAKE_SOURCE_DIR}/config
    ${CMAKE_SOURCE_DIR}/app
    ${CMAKE_SOURCE_DIR}/bsp
    ${CMAKE_SOURCE_DIR}/drivers
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F
)

target_link_options(robot_firmware PRIVATE
    ${MCU_FLAGS}
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -T${CMAKE_SOURCE_DIR}/config/memory.ld
    -nostdlib
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/robot_firmware.map
)

target_link_libraries(robot_firmware PRIVATE gcc)

set_target_properties(robot_firmware PROPERTIES SUFFIX ".elf")

add_custom_command(TARGET robot_firmware POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary
        $<TARGET_FILE:robot_firmware>
        ${CMAKE_CURRENT_BINARY_DIR}/robot_firmware.bin
)