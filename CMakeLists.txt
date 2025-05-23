cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME vbcore_stepper_spi)

include("cmake/gcc-arm-none-eabi.cmake")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

enable_language(C ASM)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

file(GLOB_RECURSE USER_SOURCES "App/*.cpp")
file(GLOB_RECURSE LIBCXXCANARD_SOURCES "Drivers/libcxxcanard/cyphal/*.cpp" "Drivers/libcxxcanard/libs/*.c")
file(GLOB_RECURSE LIBVOLTBRO_SOURCES "Drivers/libvoltbro/*.cpp" "Drivers/libvoltbro/*.c")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
    ${LIBVOLTBRO_SOURCES}
    ${LIBCXXCANARD_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App
    Drivers/libcxxcanard
    Drivers/libcxxcanard/libs
    Drivers/libvoltbro
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    -D__FPU_PRESENT
)

add_custom_command(
    POST_BUILD
    TARGET ${CMAKE_PROJECT_NAME}
    COMMAND arm-none-eabi-objcopy -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
)
