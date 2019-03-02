CC=g++

SRCS=wrapper.cpp
HDRS=wrapper.h quickqueue.h serialman.h periodic.h msg_defs.h circular_buffer.h

test: ${SRCS} ${HDRS}
	g++ -std=c++17 -Iserial/include main.cpp ${SRCS} -o bin/test -Lserial/build -lserialc

libarticulate: ${SRCS} ${HDRS}
	g++ -std=c++17 -shared -o articulate.dll -Iserial/include ${SRCS} -Lserial/build -lserialc