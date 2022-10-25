CXX=g++ -std=c++11
OPT=-O3

ETA: main.o Alg1.o Alg2.o od_generation.o structure.o -lpthread -lboost_system -lboost_thread
	$(CXX) -g -o ETA main.o Alg1.o Alg2.o od_generation.o structure.o -lpthread -lboost_system -lboost_thread

main.o:main.cpp
	$(CXX) -g -c $(OPT) main.cpp
Alg1.o:Alg1.cpp
	$(CXX) -g -c $(OPT) Alg1.cpp
Alg2.o:Alg2.cpp
	$(CXX) -g -c $(OPT) Alg2.cpp
od_generation.o:od_generation.cpp
	$(CXX) -g -c $(OPT) od_generation.cpp
structure.o:structure.cpp
	$(CXX) -g -c $(OPT) structure.cpp
	
clean:
	rm *.o
	rm DKSP
