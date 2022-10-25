#CXX=g++ -std=c++11
CXX=//home/data/zizhuo/anaconda3/bin/x86_64-conda-linux-gnu-g++ -std=c++11
OPT=-O3

INC=-I/home/data/zizhuo/anaconda3/include

#ETA: main.o Alg1.o Alg2.o od_generation.o structure.o -lpthread -lboost_system -lboost_thread
#	$(CXX) -g -o ETA main.o Alg1.o Alg2.o od_generation.o structure.o -lpthread -lboost_system -lboost_thread
ETA: main.o Alg1.o Alg2.o od_generation.o structure.o -lpthread -lboost_system -lboost_thread /home/data/zizhuo/anaconda3/lib/libstdc++.so.6
	$(CXX) -g -o ETA main.o Alg1.o Alg2.o od_generation.o structure.o -lpthread /home/data/zizhuo/anaconda3/lib/libboost_system.so /home/data/zizhuo/anaconda3/lib/libboost_thread.so 

main.o:main.cpp
	$(CXX) -g -c $(OPT) $(INC) main.cpp
Alg1.o:Alg1.cpp
	$(CXX) -g -c $(OPT) $(INC) Alg1.cpp
Alg2.o:Alg2.cpp
	$(CXX) -g -c $(OPT) $(INC) Alg2.cpp
od_generation.o:od_generation.cpp
	$(CXX) -g -c $(OPT) $(INC) od_generation.cpp
structure.o:structure.cpp
	$(CXX) -g -c $(OPT) $(INC) structure.cpp
	
clean:
	rm *.o
	rm ETA
