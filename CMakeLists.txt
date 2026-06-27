cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RC_RobotDog_Rebuild20260605)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
#        Development_Layer/External_Device/Motor/Motor_DM.cpp
#        Development_Layer/External_Device/Motor/Motor_DM.h
#        Development_Layer/Driver/CAN/drv_can.cpp
#        Development_Layer/Driver/CAN/drv_can.h
#        Development_Layer/Robot_Control/Controller/PID.cpp
#        Development_Layer/Robot_Control/Controller/PID.h
#        Development_Layer/Driver/Math/drv_math.cpp
#        Development_Layer/Driver/Math/drv_math.h
#        Development_Layer/Function_Class/Class_RobotDog/robotdog_class.cpp
#        Development_Layer/Function_Class/Class_RobotDog/robotdog_class.h
#        Development_Layer/Robot_Control/Foot_Control/foot_control.cpp
#        Development_Layer/Robot_Control/Foot_Control/foot_control.h
#        Development_Layer/Task/CallBack_Task/callback_task.cpp
#        Development_Layer/Task/CallBack_Task/callback_task.h
#        Development_Layer/External_Device/FS-IA6B/fs_ia6b.cpp
#        Development_Layer/External_Device/FS-IA6B/fs_ia6b.h
#        Development_Layer/Driver/UART/drv_uart.cpp
#        Development_Layer/Driver/UART/drv_uart.h
#        Development_Layer/Driver/Tim/drv_tim.cpp
#        Development_Layer/Driver/Tim/drv_tim.h
        Development_Layer/External_Device/Motor/Motor_DM.cpp
        Development_Layer/Driver/CAN/drv_can.cpp
        Development_Layer/Robot_Control/Controller/PID.cpp
        Development_Layer/Driver/Math/drv_math.cpp
        Development_Layer/Function_Class/Class_RobotDog/robotdog_class.cpp
        Development_Layer/Robot_Control/Foot_Control/foot_control.cpp
        Development_Layer/Task/CallBack_Task/callback_task.cpp
        Development_Layer/External_Device/FS-IA6B/fs_ia6b.cpp
        Development_Layer/Driver/UART/drv_uart.cpp
        Development_Layer/Driver/Tim/drv_tim.cpp
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

        Development_Layer/External_Device/Motor
        Development_Layer/Driver/CAN
        Development_Layer/Robot_Control/Controller
        Development_Layer/Driver/Math
        Development_Layer/Function_Class/Class_RobotDog
        Development_Layer/Robot_Control/Foot_Control
        Development_Layer/Task/CallBack_Task
        Development_Layer/External_Device/FS-IA6B
        Development_Layer/Driver/UART
        Development_Layer/Driver/Tim
        Drivers/CMSIS/DSP/Include

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
