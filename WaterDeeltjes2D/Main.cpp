#include <stdio.h>
#include <cmath>
#include <iostream>
#include "math2D.h"
#include "waterDeeltje.h"
#include <chrono>

int main() {
	int Steps = 5000;

	//ParticleSystemB* deeltjesSys = new ParticleSystemB(16, 1.1f);
	// arg: N_particles, r_cutoff

	//SimulatorB* Simu = new SimulatorB(10000.0f, 1.1f, 1.05f, 100.0f, 0.005f);
	// arg: spring constant, spring damping, string tension, spring stress (angle bending), timestep

	ParticleSys1D* deeltjesSys = new ParticleSys1D(16);
	// arg: N_particles, r_cutoff

	Simulator1D* Simu = new Simulator1D(10.0f, 0.01f, 0.005f);
	// arg: spring constant, spring damping, string tension, spring stress (angle bending), timestep

	std::cout << "How many timesteps?" << std::endl;
	std::cin >> Steps;
	// RUN SOLVER and time it
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	for (int i = 0; i < Steps; i++) {
		Simu->verletSolver(deeltjesSys);
		std::cout << "pos.y of 7th particle " << deeltjesSys->positiony[7] << '\n';
	}	
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

	std::cout << "Elapsed time per step= " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()/(float)Steps << "[us]" << std::endl;

}


