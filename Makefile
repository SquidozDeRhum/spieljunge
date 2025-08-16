default: setup main.o tools.o instruction.o prefixed.o
	g++ ./build/main.o ./build/tools.o ./build/instruction.o ./build/prefixed.o -o ./build/main -Wall
	./build/main

setup:
	@if [ ! -d ./build ]; then mkdir ./build; fi

main.o: ./main.cpp
	g++ -c ./main.cpp -o ./build/main.o

tools.o: ./lib/tools.cpp
	g++ -c ./lib/tools.cpp -o ./build/tools.o

instruction.o: ./lib/instruction.cpp
	g++ -c ./lib/instruction.cpp -o ./build/instruction.o

prefixed.o: ./lib/prefixed.cpp
	g++ -c ./lib/prefixed.cpp -o ./build/prefixed.o