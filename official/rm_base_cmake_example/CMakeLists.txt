#cmake最小需要版本
cmake_minimum_required(VERSION 2.8)
#项目名字
project(RM_CMAKE_EXAMPLE)
#包含原程序,即把给定目录下的源程序复制给变量DIR_SRC
aux_source_directory(DIR_SRC ./)
INCLUDE_DIRECTORIES(./linux-x86/include)

#find_library(RM_API libRM_API ./linux-x86)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/linux-x86/lib)
#生成程序
add_executable(rm_camke_example main.cpp)

target_link_libraries(rm_camke_example libRM_Base.so)


