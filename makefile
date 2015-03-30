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
LDFLAGS			+= 	-lboost_program_options-mt -lpthread -lvolk
env				 	= 	LD_LIBRARY_PATH="/target/lib/"

objs			:=			\
  thread-main.o		\
	rx-thread.o			\
	tx-thread.o			\

all							: thread-main.exe rx-thread.exe tx-thread.exe

thread-main.exe	: thread-main.o
	$(env) $(CXX) $^ -o $@ $(LDFLAGS)

thread-main.o		: thread-main.cc
	$(CXX) $(CFLAGS) -c $< -o $@

rx-thread.exe		: rx-thread.o
	$(env) $(CXX) $^ -o $@ $(LDFLAGS)

rx-thread.o			: rx-thread.cc
	$(CXX) $(CFLAGS) -c $< -o $@

tx-thread.exe		: tx-thread.o
	$(env) $(CXX) $^ -o $@ $(LDFLAGS)

tx-thread.o			: tx-thread.cc
	$(CXX) $(CFLAGS) -c $< -o $@

clean			:
	$(RM) $(objs)