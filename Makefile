fast:
	clang++ main.cpp -std=c++17 -O3 -Isrc/ -I${GUROBI_HOME}/include/ -L${GUROBI_HOME}/lib -lgurobi_c++ -lgurobi91
	./a.out <test/F-n135-k7.vrp
debug:
	clang++ main.cpp -std=c++17 -g -fsanitize=address -Isrc/ -I${GUROBI_HOME}/include/ -L${GUROBI_HOME}/lib -lgurobi_c++ -lgurobi91
	./a.out <test/F-n135-k7.vrp