#include <stdio.h>
#include <cmath>
#include <iostream>
#include "math2D.h"
#include "waterDeeltje.h"
#include <chrono>

int main() {
	int Steps = 5000;

	ParticleSystem* deeltjesSys = new ParticleSystem(16,1.1f); // arg: sqrt(N), r_cutoff
	Simulator* Simu = new Simulator(1.0f, 0.01f, 0.005f); // arg: spring constant, spring damping, timestep
	
	std::cout << "How many timesteps?" << std::endl;
	std::cin >> Steps;
	// RUN SOLVER and time it
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	for (int i = 0; i < Steps; i++) {
		Simu->verletSolver(deeltjesSys);
		//std::cout << deeltjesSys->WaterDeeltjes[6].position.x << '\n'; // check the result
	}	
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

	std::cout << "Elapsed time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

	//for (int i = 0; i < 16; i++) {
	//	std::cout << "particle " << i << " force x-component " << deeltjesSys->WaterDeeltjes[i].force.x << '\n';
	//	std::cout << "particle " << i << " force y-component " << deeltjesSys->WaterDeeltjes[i].force.y << '\n';
	//}
}


