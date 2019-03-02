cmake_minimum_required(VERSION 2.8.3)
project(articul8)

set(CMAKE_CXX_STANDARD 17)

## Sources
set(articul8_SRCS
    wrapper.cpp
    	wrapper.h 
	quickqueue.h 
	serialman.h 
	periodic.h 
	msg_defs.h 
	circular_buffer.h
)
	
add_library(articul8 SHARED ${articul8_SRCS})

find_library(serialc_LIBRARY serialc serial/build)
target_link_libraries(articul8 ${serialc_LIBRARY})

set_target_properties(articul8 PROPERTIES PUBLIC_HEADER wrapper.h)
include_directories(serial/include)
