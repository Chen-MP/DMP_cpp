cmake_minimum_required(VERSION 3.7)
project(CanonicalSystem)

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
# 可执行文件 CanonicalSystem
add_executable(CanonicalSystem cs.cpp Canonical_System/CanonicalSystem.cpp)
target_link_libraries(CanonicalSystem PRIVATE ${Python3_LIBRARIES})

# 可执行文件 discrete_psi
add_executable(discrete_psi discrete_psi.cpp 
                            Canonical_System/CanonicalSystem.cpp
                            Original_discrete/original_discrete.cpp)
target_link_libraries(discrete_psi PRIVATE ${Python3_LIBRARIES})

# 可执行文件 discrete_dmp
add_executable(discrete_dmp discrete_dmp.cpp 
                            Canonical_System/CanonicalSystem.cpp
                            Original_discrete/original_discrete.cpp)
target_link_libraries(discrete_dmp PRIVATE ${Python3_LIBRARIES})

# 可执行文件 dis_dmp_obstacle
add_executable(dis_dmp_obstacle dis_DMP_obstacle.cpp 
                                Canonical_System/CanonicalSystem.cpp
                                Original_discrete/original_discrete.cpp)
target_link_libraries(dis_dmp_obstacle PRIVATE ${Python3_LIBRARIES})

# 可执行文件 plotscatter
add_executable(plotscatter test.cpp)
target_link_libraries(plotscatter PRIVATE ${Python3_LIBRARIES})

add_definitions("-Wall -g")