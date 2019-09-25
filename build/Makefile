exe: main.o grid_input.o Astar.o Astar_getneighbor.o obb.o opencvdraw.o Collision_checking.o execute.o bezier_curve.o path_process.o obstacle.o obs_create.o
	g++ -o exe.out main.o grid_input.o Astar.o Astar_getneighbor.o execute.o obb.o opencvdraw.o collision_checking.o bezier_curve.o path_process.o obstacle.o obs_create.o $(shell pkg-config --cflags --libs opencv)

main.o: ../main.cpp 
	g++ -c ../main.cpp -Wall -std=c++11

grid_input.o: ../include/grid_input.cpp
	g++ -c ../include/grid_input.cpp -std=c++11

Astar.o: ../include/Astar.cpp
	g++ -c ../include/Astar.cpp -std=c++11

Astar_getneighbor.o: ../include/Astar_getneighbor.cpp
	g++ -c ../include/Astar_getneighbor.cpp -std=c++11

obb.o: ../include/obb.cpp
	g++ -c ../include/obb.cpp -std=c++11

opencvdraw.o: ../include/opencvdraw.cpp
	g++ -c ../include/opencvdraw.cpp -std=c++11

bezier_curve.o: ../include/bezier_curve.cpp
	g++ -c ../include/bezier_curve.cpp -std=c++11

Collision_checking.o: ../include/collision_checking.cpp
	g++ -c ../include/collision_checking.cpp -std=c++11

execute.o: ../include/execute.cpp
	g++ -c ../include/execute.cpp -std=c++11

path_process.o: ../include/path_process.cpp
	g++ -c ../include/path_process.cpp -std=c++11

obs_create.o: ../include/obs_create.cpp
	g++ -c ../include/obs_create.cpp -std=c++11

obstacle.o: ../include/obstacle.cpp
	g++ -c ../include/obstacle.cpp -std=c++11


clean: 
	rm -f *.o 