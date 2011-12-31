CC=g++
CXXFLAGS=-std=c++0x -ggdb -Wall -Wextra -O0 -fmax-errors=15 -I/usr/include/SDL/ -lSDL -lGL -lGLU

ODIR=output

DEPS = world.hpp utils.hpp polygon_collision_detection.hpp bbox_collision_detector.hpp
_OBJ = main.o mobile_objects.o polygon_collision_detection.o the_decomposition_of_the_world_into_blocks.o tile_changing_implementations.o water.o world_building.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CXXFLAGS)

lasercake: $(OBJ)
	$(CC) -o $@ $^ $(CXXFLAGS)
