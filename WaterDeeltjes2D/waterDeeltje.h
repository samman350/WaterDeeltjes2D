#pragma once

#include <cmath>
#include <iostream>
#include "math2D.h"
#include <vector>
#include <random>

class waterDeeltje {
public:
	Vec2 position;
	Vec2 velocity;
	Vec2 oldAcceleration = Vec2(0.0f,0.0f);
	Vec2 newAcceleration; // de verschillende definities hebben met solver te maken
	Vec2 force = Vec2(0.0f, 0.0f);
	std::vector<waterDeeltje> neighbours = std::vector<waterDeeltje>();
	float mass;
	bool simulable; // is the particle motion dictated by forces or by finger?

	waterDeeltje(Vec2 initPos, Vec2 initVel, Vec2 initAcc, float massParticle):
		position(initPos), // initializer list
		velocity(initVel),
		newAcceleration(initAcc),
		mass(massParticle)
		{}
};

class ParticleSystem {
public:
	std::vector<waterDeeltje> WaterDeeltjes;

	ParticleSystem(const int N_Particles, const float r_cutoff) {
		
		WaterDeeltjes.reserve(N_Particles);

		// maak een NxN grid van deeltjes
		for (int i = 0; i < N_Particles; i++ ) {
			for (int j = 0; j < N_Particles; j++ ) {
				WaterDeeltjes.emplace_back(Vec2((float)i, (float)j),
					Vec2(0.0f, 0.0f),
					Vec2(0.0f, 0.0f),
						1.0f);
			}
		}
		
		// vul de burenlijst (als afstand kleiner dan cutoff dan buren)
		for (int i = 0; i < N_Particles * N_Particles; i++) {
			for (int j = 0; j < N_Particles * N_Particles; j++) {
				if (i!=j && WaterDeeltjes[i].position.DistanceTo(WaterDeeltjes[j].position) < r_cutoff) {
					WaterDeeltjes[i].neighbours.emplace_back(WaterDeeltjes[j]);
				}
			}
		}
	}
};

class Simulator {
public:
	float distance;
	float offset;
	Vec2 direction = Vec2(0.0f, 0.0f);
	float springConstant;
	float springDamping;
	float timeStep;
	ParticleSystem deeltjesSysteem;

	void verletSolver(ParticleSystem* ps, float springcst, float springdmp, float timestep) {
		updatePositions(ps, timestep);
		updateForces(ps, springcst, springdmp);
		updateAccelerations(ps);
		updateVelocities(ps, timestep);
	}

private:

	void calculateForces(waterDeeltje& eneDeeltje, float springCst, float springDmp) {
		for (int i = 0; i < eneDeeltje.neighbours.size(); i++) {

		}
	}

	void updatePositions(ParticleSystem * deeltjessysteem, float timestep) {
		for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
			deeltje.position = deeltje.position 
				+ deeltje.velocity * timestep 
				+ deeltje.newAcceleration * timestep * timestep * 0.5;
		}
	}

	void updateForces(ParticleSystem* deeltjessysteem, float springCsnt, float springDmper) {
		for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
			calculateForces(deeltje, springCsnt, springDmper);
		}
	}

	void updateAccelerations(ParticleSystem* deeltjessysteem) {
		for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
			deeltje.oldAcceleration = deeltje.newAcceleration; // oud wordt nieuw; nieuw schuift door naar oud
			deeltje.newAcceleration = deeltje.force / deeltje.mass;
		}
	}

	void updateVelocities(ParticleSystem* deeltjessysteem, float timestep) {
		for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
			deeltje.velocity = deeltje.velocity
				+ (deeltje.oldAcceleration + deeltje.newAcceleration) * timestep * 0.5;
		}
	}
};