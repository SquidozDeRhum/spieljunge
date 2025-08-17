default: setup main.o tools.o instruction.o prefixed.o
	g++ ./build/main.o ./build/tools.o ./build/instruction.o ./build/prefixed.o -o ./build/main -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -Wall
	./build/main

setup:
	@if [ ! -d ./build ]; then mkdir ./build; fi

main.o: ./main.cpp
	g++ -c ./main.cpp -o ./build/main.o

tools.o: ./src/tools.cpp
	g++ -c ./src/tools.cpp -o ./build/tools.o

instruction.o: ./src/instruction.cpp
	g++ -c ./src/instruction.cpp -o ./build/instruction.o

prefixed.o: ./src/prefixed.cpp
	g++ -c ./src/prefixed.cpp -o ./build/prefixed.o