all: getmymatrices

CFLAGS=-ansi -D_DONT_USE_STACK_GUARD -O3
OBJFILES=Main.o Dir.o NewFix.o PNGLoad.o HTTP_Client.o HTTP_Download.o URI.o Socket.o StlUtils.o CallStack.o Thread.o Timer.o UniBlocks.o Unicode.o UniConv.o UniTables.o XML.o

getmymatrices: $(OBJFILES)
	g++ -pthread -lrt $(CFLAGS) -o getmymatrices $(OBJFILES) -lpng

Main.o: Main.cpp UFLSMC.h
	g++ $(CFLAGS) -c Main.cpp -o Main.o

%.o: %.cpp
	g++ $(CFLAGS) -c $< -o $@

%.o: ../UberLame_src/%.cpp
	g++ $(CFLAGS) -c $< -o $@

%.o: ../UberLame_src/proto/%.cpp
	g++ $(CFLAGS) -c $< -o $@

%.o: ../UberLame_src/iface/%.cpp
	g++ $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o getmymatrices
