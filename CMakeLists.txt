cmake_minimum_required(VERSION 3.10.2)

project(hgsd)

# Prepare the build environment
set(ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

# Add your project files
include_directories("include/")
set(SRC_DIR "src/")

list(APPEND SRCFILES "${SRC_DIR}/Drone.cpp")
list(APPEND SRCFILES "${SRC_DIR}/DVRPD.cpp")
list(APPEND SRCFILES "${SRC_DIR}/GeneticAlgorithm.cpp")
list(APPEND SRCFILES "${SRC_DIR}/Vehicle.cpp")
list(APPEND SRCFILES "${SRC_DIR}/Routes.cpp")
list(APPEND SRCFILES "${SRC_DIR}/Location.cpp")
list(APPEND SRCFILES "${SRC_DIR}/Tests.cpp")
#list(APPEND SRCFILES main.cpp)

# create executable
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


#set lib dir
set (LIB_DIR "/lib")
set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -fno-omit-frame-pointer -fsanitize=undefined -fsanitize=address")
set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -Wall -Wextra -fno-omit-frame-pointer -fsanitize=undefined -fsanitize=address")

message("Search path for libs set to ${LIB_DIR}.")
	
# create project target - win: allow visual studio posix functions
add_executable(${PROJECT_NAME} ${SRCFILES})
target_compile_definitions(${PROJECT_NAME} PUBLIC _CRT_SECURE_NO_WARNINGS)

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)