default: setup main.o tools.o instruction.o prefixed.o rendering.o imgui.o imgui_draw.o imgui_tables.o imgui_widgets.o rlImGui.o
	g++ ./build/main.o ./build/tools.o ./build/instruction.o ./build/prefixed.o ./build/rendering.o ./build/imgui.o ./build/imgui_draw.o ./build/imgui_tables.o ./build/imgui_widgets.o ./build/rlImGui.o -o ./build/main -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -Wall
	./build/main

setup:
	@if [ ! -d ./build ]; then mkdir ./build; fi

main.o: ./main.cpp
	g++ -c ./main.cpp -o ./build/main.o -Ithird_party/imgui -Ithird_party/rlImgui

tools.o: ./src/tools.cpp
	g++ -c ./src/tools.cpp -o ./build/tools.o

instruction.o: ./src/instruction.cpp
	g++ -c ./src/instruction.cpp -o ./build/instruction.o

prefixed.o: ./src/prefixed.cpp
	g++ -c ./src/prefixed.cpp -o ./build/prefixed.o

rendering.o: ./src/rendering.cpp
	g++ -c ./src/rendering.cpp -o ./build/rendering.o

imgui.o: ./third_party/imgui/imgui.cpp
	g++ -c ./third_party/imgui/imgui.cpp -o ./build/imgui.o -Ithird_party/imgui

imgui_draw.o: ./third_party/imgui/imgui_draw.cpp
	g++ -c ./third_party/imgui/imgui_draw.cpp -o ./build/imgui_draw.o -Ithird_party/imgui

imgui_tables.o: ./third_party/imgui/imgui_tables.cpp
	g++ -c ./third_party/imgui/imgui_tables.cpp -o ./build/imgui_tables.o -Ithird_party/imgui

imgui_widgets.o: ./third_party/imgui/imgui_widgets.cpp
	g++ -c ./third_party/imgui/imgui_widgets.cpp -o ./build/imgui_widgets.o -Ithird_party/imgui

rlImGui.o: ./third_party/rlImgui/rlImGui.cpp
	g++ -c ./third_party/rlImgui/rlImGui.cpp -o ./build/rlImGui.o -Ithird_party/imgui