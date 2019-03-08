cmake_minimum_required(VERSION 2.8.3)
project(articul8)

set(CMAKE_CXX_STANDARD 17)

## Sources
set(articul8_SRCS
    wrapper/wrapper.cpp
    wrapper/wrapper.h 
    wrapper/logger.cpp
    wrapper/logger.h
	wrapper/quickqueue.h 
	wrapper/serialman.h 
	wrapper/periodic.h 
	wrapper/msg_defs.h 
	wrapper/circular_buffer.h
)

set(test_SRCS
	wrapper/logger.cpp
	wrapper/logger.h
	wrapper/tests/logtest.cpp
)

add_library(articul8 SHARED ${articul8_SRCS})
add_executable(testLog ${test_SRCS})

find_library(serialc_LIBRARY serialc serial/build)
target_link_libraries(articul8 ${serialc_LIBRARY})

set_target_properties(articul8 PROPERTIES PUBLIC_HEADER wrapper.h)
include_directories(serial/include wrapper)

