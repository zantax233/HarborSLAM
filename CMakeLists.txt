cmake_minimum_required(VERSION 2.8)
project(HarborSLAM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(Eigen3 REQUIRED )
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

include_directories(
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Sophus_INCLUDE_DIRS}
	${FMT_INCLUDE_DIRS}
	"usr/local/eigen3"
)

link_directories(
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED
			src/Frame.cpp
			src/Plane.cpp
			src/Track.cpp
			src/Utils.cpp)
target_link_libraries(${PROJECT_NAME} 
			${PCL_LIBRARIES} 
			${OpenCV_LIBRARIES} 
			Eigen3::Eigen 
			${Sophus_LIBRARIES} 
			fmt)

add_executable(runHarborSLAM examples/main.cpp)
target_link_libraries(runHarborSLAM ${PROJECT_NAME})
