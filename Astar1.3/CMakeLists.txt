cmake_minimum_required( VERSION 2.8 )
project( Astar1.3 )

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )


# 工程内的库
add_library( Astar Astar.cpp )
add_library( imageDisplay imageDisplay.cpp )

# 寻找OpenCV库
find_package( OpenCV 3.1 REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( main main.cpp )
# 将库文件链接到可执行程序上
target_link_libraries( main Astar )
# target_link_libraries( main imageDisplay )
target_link_libraries( main ${OpenCV_LIBS} )
