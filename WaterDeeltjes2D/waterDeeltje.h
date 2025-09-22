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
	Vec2 newAcceleration; // de verschillende definities van acc hebben met solver te maken

	Vec2 force = Vec2(0.0f, 0.0f);
	std::vector<waterDeeltje*> neighbours = std::vector<waterDeeltje*>();
	std::vector<float> offsets = std::vector<float>();
	float mass;
	bool simulable; // is the particle motion dictated by forces (true), or static/other (false)?

	waterDeeltje(Vec2 initPos, Vec2 initVel, Vec2 initAcc, float massParticle, bool simulatable):
		position(initPos), // initializer list
		velocity(initVel),
		newAcceleration(initAcc),
		mass(massParticle),
		simulable(simulatable)
		{}
};

class ParticleSystem {
public:
	std::vector<waterDeeltje> WaterDeeltjes;

	ParticleSystem(const int N_Particles, const float r_cutoff) {
		
		for (int i = 0; i < N_Particles; i++) {
			if (i == 0 || i == N_Particles - 1) {
				WaterDeeltjes.emplace_back(Vec2((float)i, 0.0f),
					Vec2(0.0f, 0.0f),
					Vec2(0.0f, 0.0f),
					1.0f,
					false); // fix outer particles in grid

			}
			else if (i == 6) {
				WaterDeeltjes.emplace_back(Vec2((float)i, 0.0f),
					Vec2(0.0f, 1.0f),
					Vec2(0.0f, 0.0f),
					1.0f,
					true);
			}
			else {
				WaterDeeltjes.emplace_back(Vec2((float)i, 0.0f),
					Vec2(0.0f, 0.0f),
					Vec2(0.0f, 0.0f),
					1.0f,
					true);
			}
		}
		// vul de burenlijst (als afstand kleiner dan cutoff dan buren)
		for (int i = 0; i < N_Particles; i++) {
			for (int j = 0; j < N_Particles; j++) {
				if (i != j && WaterDeeltjes[i].position.DistanceTo(WaterDeeltjes[j].position) < r_cutoff) {
					WaterDeeltjes[i].neighbours.emplace_back(&WaterDeeltjes[j]); // fill neighbour list with memory adresses of neighbours
					WaterDeeltjes[i].offsets.emplace_back(
					WaterDeeltjes[i].position.DistanceTo(WaterDeeltjes[j].position)); // fill offsets list
				}
			}
			std::cout << "particle " << i << " has " << WaterDeeltjes[i].neighbours.size() << " neighbours " << std::endl;
		}
	}
};

class Simulator {
public:
	
	float springConstant;
	float springDamping;
	float timeStep;
	
	Simulator(float SpringConst, float SpringDamp, float TimeStep) :
		springConstant(SpringConst),
		springDamping(SpringDamp),
		timeStep(TimeStep)
		{};

	void verletSolver(ParticleSystem* ps) {
		updatePositions(ps, timeStep);
		updateForces(ps, springConstant, springDamping);
		updateAccelerations(ps);
		updateVelocities(ps, timeStep);
	}

private:

	float distance = 0.0f;
	float offset = 0.0f;
	Vec2 delta_v = Vec2(0.0f , 0.0f);
	Vec2 direction = Vec2(0.0f , 0.0f);

	void calculateForces(waterDeeltje& eneDeeltje, const float springCst, const float springDmp) {
		eneDeeltje.force = Vec2(0.0f, 0.0f); // set force of previous timestep to zero
		for (int i = 0; i < eneDeeltje.neighbours.size(); i++) {
			distance = eneDeeltje.position.DistanceTo(eneDeeltje.neighbours[i]->position); // REMEMBER the array neighbnours contains pointers!
			direction = eneDeeltje.position.DirectionTo(eneDeeltje.neighbours[i]->position);
			offset = eneDeeltje.offsets[i];
			delta_v = eneDeeltje.velocity - eneDeeltje.neighbours[i]->velocity;
			eneDeeltje.force = eneDeeltje.force - direction * (distance - offset) * springCst
				- direction*springDmp*(delta_v.Dot(direction)); // last term is damping
		}
	}

	void updatePositions(ParticleSystem * deeltjessysteem, const float timestep) {
		for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
			if (deeltje.simulable) {
				deeltje.position = deeltje.position
					+ deeltje.velocity * timestep
					+ deeltje.newAcceleration * timestep * timestep * 0.5;
			}
		}
	}

	void updateForces(ParticleSystem* deeltjessysteem, const float springCsnt, const float springDmper) {
		for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
			if (deeltje.simulable) {
				calculateForces(deeltje, springCsnt, springDmper);
			}

		}
	}

	void updateAccelerations(ParticleSystem* deeltjessysteem) {
		for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
			if (deeltje.simulable) {
				deeltje.oldAcceleration = deeltje.newAcceleration; // oud wordt nieuw; nieuw schuift door naar oud
				deeltje.newAcceleration = deeltje.force / deeltje.mass;
			}
			
		}
	}

	void updateVelocities(ParticleSystem* deeltjessysteem, const float timestep) {
		for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
			if (deeltje.simulable) {
				deeltje.velocity = deeltje.velocity
					+ (deeltje.oldAcceleration + deeltje.newAcceleration) * timestep * 0.5;
			}
		}
	}
};