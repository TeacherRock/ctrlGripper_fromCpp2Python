# CMakeList.txt: 最上層的 CMake 專案檔，執行全域設定
# 並在這裡包含子專案。
#
cmake_minimum_required (VERSION 3.8)

project ("ctrlGripper_py")

set( basic_cxx17 "c++17" )
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(Boost_INCLUDE_DIR "D:\\Group\\NSTC2024\\Robotiq_Gripper\\boost_1_64_0_x64")
set(Boost_LIBRARY_DIR "D:\\Group\\NSTC2024\\Robotiq_Gripper\\boost_1_64_0_x64\\lib64-msvc-14.1")

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

set(dll_SOURCE_FILES
    ctrlGripper.cpp
	MyRobot_AsyncSerial.cpp 
    MyRobot_Protocol.cpp
    ROBOTIQ_function.cpp
    stdafx.cpp
)

add_library(ctrlGripper_dll SHARED ${dll_SOURCE_FILES})
target_link_libraries( ctrlGripper_dll ${Boost_LIBRARIES} )

add_executable (main_gripper main.cpp)

target_link_libraries( main_gripper ctrlGripper_dll )
