#include <stdio.h>
#include <cmath>
#include <iostream>
#include "math2D.h"
#include "waterDeeltje.h"

int main() {
	int Steps = 1000;

	ParticleSystem* deeltjesSys = new ParticleSystem(4,1.1f); // arg: sqrt(N), r_cutoff
	Simulator* Simu = new Simulator(1.0f, 0.1f, 0.005f); // arg: spring constant, spring damping, timestep
	
	// RUN SOLVER
	for (int i = 0; i < Steps; i++) {
		Simu->verletSolver(deeltjesSys);
		std::cout << deeltjesSys->WaterDeeltjes[6].position.x << std::endl;
	}
	
}


