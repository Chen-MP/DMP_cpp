cmake_minimum_required(VERSION 3.7)
project(canonical_system)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 Python3
find_package(Python3 REQUIRED COMPONENTS Development)
include_directories(${Python3_INCLUDE_DIRS})

# 查找 matplotlib-cpp
find_path(MATPLOTLIB_CPP_INCLUDE_DIR matplotlibcpp.h)
if(MATPLOTLIB_CPP_INCLUDE_DIR)
    include_directories(${MATPLOTLIB_CPP_INCLUDE_DIR})
    message(STATUS "Found matplotlib-cpp at: ${MATPLOTLIB_CPP_INCLUDE_DIR}")  # 打印路径
else()
    message(WARNING "matplotlib-cpp not found!")
endif()

# 确保当前目录包含头文件
include_directories(.)

# ${Python3_LIBRARIES} 是由 find_package(Python3) 自动生成的变量
# 可执行文件 canonical_system
add_executable(canonical_system demo/cs.cpp canonical_system/canonical_system.cpp)
target_link_libraries(canonical_system PRIVATE ${Python3_LIBRARIES})

# 可执行文件 discrete_psi
add_executable(discrete_psi demo/discrete_psi.cpp 
                            canonical_system/canonical_system.cpp
                            original_discrete/original_discrete.cpp)
target_link_libraries(discrete_psi PRIVATE ${Python3_LIBRARIES})

# 可执行文件 discrete_dmp
add_executable(discrete_dmp demo/discrete_dmp.cpp 
                            canonical_system/canonical_system.cpp
                            original_discrete/original_discrete.cpp)
target_link_libraries(discrete_dmp PRIVATE ${Python3_LIBRARIES})

# 可执行文件 dis_dmp_obstacle
add_executable(dis_dmp_obstacle demo/dis_DMP_obstacle.cpp 
                                canonical_system/canonical_system.cpp
                                original_discrete/original_discrete.cpp)
target_link_libraries(dis_dmp_obstacle PRIVATE ${Python3_LIBRARIES})

# 可执行文件 plotscatter
add_executable(plotscatter demo/test.cpp)
target_link_libraries(plotscatter PRIVATE ${Python3_LIBRARIES})

add_definitions("-Wall -g")