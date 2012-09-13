default: all

all: hubo-main hubo-default hubo-console

CFLAGS := -I./include -g --std=gnu99
CXXFLAGS := -I./include -g

CC := gcc
CXX := g++


# SOCKETCAN #
CAN_LIBS :=
CAN_OBJS := src/hubo-socketcan.o
CAN_DEFS :=

# esd CAN #
# CAN_LIBS := -lntcan
# CAN_OBJS := src/hubo-esdcan.o
# CAN_DEFS := -DHUBO_CONFIG_ESD


LIBS := -lach -lrt $(CAN_LIBS)

hubo_main_objs := src/hubo-main.o $(CAN_OBJS)

hubo-main: $(hubo_main_objs)
	$(CC) -o $@  $(hubo_main_objs) $(LIBS)

hubo-default: src/hubo-default.c
	$(CC) $(CFLAGS) -o $@ $< -lach

hubo-console: src/hubo-console.o
	$(CXX)  -o $@ $< -lach -lreadline

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<


clean:
	rm -f hubo-main hubo-default hubo-console src/*.o
