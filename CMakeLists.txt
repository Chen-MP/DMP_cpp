cmake_minimum_required(VERSION 3.7)

project(CanonicalSystem)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(F:\\learn_cpp_library\\matplotlib-cpp)

include_directories(F:\\Python38\\include)
include_directories(F:\\Python38\\Lib\\site-packages\\numpy\\core\\include)

# 多文件， 这个很重要
# 确保编译器在查找包含文件时会搜索当前目录
include_directories(.)

link_directories(F:\\Python38\\libs)

# Canonical_System
add_executable(CanonicalSystem cs.cpp 
                               Canonical_System/CanonicalSystem.cpp)

target_link_libraries(CanonicalSystem python3.lib
                                      python38.lib
                                      _tkinter.lib)

# generate Guassian_function base on Canonical_System 
add_executable(discrete_psi discrete_psi.cpp 
                            Canonical_System/CanonicalSystem.cpp
                            Original_discrete/original_discrete.cpp)
       
target_link_libraries(discrete_psi python3.lib
                                   python38.lib
                                   _tkinter.lib)

# generate DMP_track base on Original_Discrete_DMP
add_executable(discrete_dmp discrete_dmp.cpp 
                            Canonical_System/CanonicalSystem.cpp
                            Original_discrete/original_discrete.cpp)
                  
target_link_libraries(discrete_dmp python3.lib
                                   python38.lib
                                   _tkinter.lib)

# generate DMP_track base on Original_Discrete_DMP and avoiding obstacle
add_executable(dis_dmp_obstacle dis_DMP_obstacle.cpp 
                                Canonical_System/CanonicalSystem.cpp
                                Original_discrete/original_discrete.cpp)

target_link_libraries(dis_dmp_obstacle python3.lib
                                       python38.lib
                                       _tkinter.lib)

add_executable(plotscatter test.cpp 
)

target_link_libraries(plotscatter python3.lib
                                       python38.lib
                                       _tkinter.lib)
add_definitions("-Wall -g")