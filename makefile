# paths
srcdir			= .

# programs
CC				:= gcc
CXX				:= g++
MV				:= mv -f
RM				:= rm -f

# flags
INCLUDE_CFLAGS	= -I./ -I/target/include
CFLAGS			= 	$(INCLUDE_CFLAGS) -g -O2 -Wall -fPIC
LDFLAGS			= 	-L/target/lib
LDFLAGS			+= 	-luhd -lliquid -lm -lc -lboost_system-mt -lboost_thread-mt -lliquidgr
LDFLAGS			+= 	-lboost_program_options-mt -lpthread
env				 	= 	LD_LIBRARY_PATH="/target/lib/"

objs			:=			\
  thread-main.o		\

all							: thread-main.exe

thread-main.exe	: thread-main.o
	$(env) $(CXX) $^ -o $@ $(LDFLAGS)

thread-main.o		: thread-main.cc
	$(CXX) $(CFLAGS) -c $< -o $@

clean			:
	$(RM) $(objs)
