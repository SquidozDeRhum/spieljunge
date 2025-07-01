default: setup main.o tools.o opcodeR.o prefixed.o
	g++ ./build/main.o ./build/tools.o ./build/opcodeR.o ./build/prefixed.o -o ./build/main -Wall
	./build/main

setup:
	@if [ ! -d ./build ]; then mkdir ./build; fi

main.o: ./main.cpp
	g++ -c ./main.cpp -o ./build/main.o

tools.o: ./lib/tools.cpp
	g++ -c ./lib/tools.cpp -o ./build/tools.o

opcodeR.o: ./lib/opcodeR.cpp
	g++ -c ./lib/opcodeR.cpp -o ./build/opcodeR.o

prefixed.o: ./lib/prefixed.cpp
	g++ -c ./lib/prefixed.cpp -o ./build/prefixed.o