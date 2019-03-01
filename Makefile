CC=g++

SRCS=main.cpp wrapper.cpp
HDRS=wrapper.h

test: ${SRCS} ${HDRS}
	g++ -std=c++17 -Iserial/include ${SRCS} -o bin/test -Lserial/build -lserialc -lpthread
