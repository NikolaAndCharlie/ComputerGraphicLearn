cmake_minimum_required(VERSION 3.12)
project(eigen_test)

set(APP_ICON IKD.ico)


set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/Debug/bin)
set(CMAKE_RUNTIME_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/Debug/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/Release/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/Release/lib)
set(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/install)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5
    COMPONENTS
	Core
	Widgets
	Gui
	Sql
	Xml
	Concurrent
	Network
	REQUIRED
)

set(Qt5_LIBS Qt5::Core Qt5::Widgets Qt5::Gui Qt5::Sql Qt5::Xml Qt5::Concurrent Qt5::Network)

file(GLOB SRCS src/*.cpp)
file(GLOB HDRS src/*.h)
set(OpenCV_DIR $ENV{OpenCV_DIR})

message($ENV{OpenCV_DIR})
# find library
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(eigen_test ${APP_RC_FILE})
if(EXISTS $ENV{Eigen_DIR})

    target_include_directories(eigen_test 
	    PRIVATE
	    ${PROJECT_SOURCE_DIR}/include/
		G:/eigen/eigen-3.4.0/
		${OpenCV_INCLUDE_DIRS}
     )
endif()


target_sources(eigen_test
    PRIVATE
	${SRCS}
	${HDRS}
	${RC}
)

target_link_libraries(eigen_test
    ${Qt5_LIBS}
	${OpenCV_LIBS}
)



