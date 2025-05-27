default: main.o tools.o opcode.o prefixed.o
	g++ ./build/main.o ./build/tools.o ./build/opcode.o ./build/prefixed.o -o ./build/main -Wall
	./build/main

main.o: ./main.cpp
	g++ -c ./main.cpp -o ./build/main.o

tools.o: ./lib/tools.cpp
	g++ -c ./lib/tools.cpp -o ./build/tools.o

opcode.o: ./lib/opcode.cpp
	g++ -c ./lib/opcode.cpp -o ./build/opcode.o

prefixed.o: ./lib/prefixed.cpp
	g++ -c ./lib/prefixed.cpp -o ./build/prefixed.o