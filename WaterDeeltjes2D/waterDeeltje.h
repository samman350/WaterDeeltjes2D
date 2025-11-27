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
    Vec2 oldAcceleration = Vec2(0.0f, 0.0f);
    Vec2 newAcceleration; // de verschillende definities van acc hebben met solver te maken

    Vec2 force = Vec2(0.0f, 0.0f);
    std::vector<waterDeeltje*> neighbours = std::vector<waterDeeltje*>();
    std::vector<float> offsets = std::vector<float>();
    float mass;
    bool simulable = true; // is the particle motion dictated by forces (true), or static/other (false)?
    bool fixed; // fixed particle?

    waterDeeltje(Vec2 initPos, Vec2 initVel, Vec2 initAcc, float massParticle, bool fixd) :
        position(initPos), // initializer list
        velocity(initVel),
        newAcceleration(initAcc),
        mass(massParticle),
        fixed(fixd)
    {}
};

class ParticleSystem {
public:
    std::vector<waterDeeltje> WaterDeeltjes;

    ParticleSystem(const int N_Particles, const float r_cutoff) {

        for (int i = 0; i < 11; i++) { // initialize the simulation
            if (i == 10) {
                WaterDeeltjes.emplace_back(Vec2((float)i, 0.0f),
                    Vec2(0.0f, 0.0f),
                    Vec2(0.0f, 0.0f),
                    1.0f,
                    true); // fix outer particles in grid

            }
            else {
                WaterDeeltjes.emplace_back(Vec2((float)i, 0.0f),
                    Vec2(0.0f, 1.0f),
                    Vec2(0.0f, 0.0f),
                    1.0f,
                    false);
            }
        }
        for (int i = 1; i < 6; i++) {
            if (i == 5) {
                WaterDeeltjes.emplace_back(Vec2(-0.5f * (float)i, squaroot3div2 * (float)i),
                    Vec2(0.0f, 0.0f),
                    Vec2(0.0f, 0.0f),
                    1.0f,
                    true); // fix outer particles in grid

            }
            else {
                WaterDeeltjes.emplace_back(Vec2(-0.5f * (float)i, squaroot3div2 * (float)i),
                    Vec2(0.0f, 0.0f),
                    Vec2(0.0f, 0.0f),
                    1.0f,
                    false);
            }
        }
        for (int i = 1; i < 5; i++) {
            if (i == 4) {
                WaterDeeltjes.emplace_back(Vec2(-0.5f * (float)i, -squaroot3div2 * (float)i),
                    Vec2(0.0f, 0.0f),
                    Vec2(0.0f, 0.0f),
                    1.0f,
                    true); // fix outer particles in grid

            }
            else {
                WaterDeeltjes.emplace_back(Vec2(-0.5f * (float)i, -squaroot3div2 * (float)i),
                    Vec2(1.0f, 0.0f),
                    Vec2(0.0f, 0.0f),
                    1.0f,
                    false);
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
        }
    }
private:
    float squaroot3div2 = 0.5f * std::sqrt(3.0f);
};

class Simulator {
public:

    const float springConstant;
    const float springDamping;
    const float stringTension;
    const float timeStep;

    Simulator(float SpringConst, float SpringDamp, float StringTension, float TimeStep) :
        springConstant(SpringConst),
        springDamping(SpringDamp),
        stringTension(StringTension),
        timeStep(TimeStep)
    {};

    void verletSolver(ParticleSystem* ps) {
        updatePositions(ps, timeStep);
        updateForcesAndAccelerations(ps, springConstant, springDamping, stringTension);
        updateVelocities(ps, timeStep);
    }

private:

    float distance = 0.0f;
    float offset = 0.0f;
    Vec2 delta_v = Vec2(0.0f, 0.0f);
    Vec2 direction = Vec2(0.0f, 0.0f);

    void calculateForcesAndAccelerations(waterDeeltje& eneDeeltje, const float springCst, const float springDmp, const float stringTns) {
        eneDeeltje.force = Vec2(0.0f, 0.0f); // set force of previous timestep to zero
        for (int i = 0; i < eneDeeltje.neighbours.size(); i++) {
            distance = eneDeeltje.position.DistanceTo(eneDeeltje.neighbours[i]->position); // REMEMBER the array neighbnours contains pointers!
            direction = eneDeeltje.position.DirectionTo(eneDeeltje.neighbours[i]->position);
            offset = eneDeeltje.offsets[i] / stringTns;
            delta_v = eneDeeltje.velocity - eneDeeltje.neighbours[i]->velocity;
            eneDeeltje.force = eneDeeltje.force - direction * (distance - offset) * springCst
                - eneDeeltje.velocity * springDmp; // last term is damping
            eneDeeltje.newAcceleration = eneDeeltje.force / eneDeeltje.mass;
        }
    }

    void updatePositions(ParticleSystem* deeltjessysteem, const float timestep) {
        const float timeStep2Div2 = 0.5f * timeStep * timeStep;
        for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
            if (!deeltje.fixed && deeltje.simulable) { // if particle is not fixed, and simulable
                deeltje.oldAcceleration = deeltje.newAcceleration; // nieuw wordt oud, nu zijn oud en nieuw gelijk
                deeltje.position = deeltje.position
                    + deeltje.velocity * timestep
                    + deeltje.newAcceleration * timeStep2Div2;
            }
        }
    }

    void updateForcesAndAccelerations(ParticleSystem* deeltjessysteem, const float springCsnt, const float springDmper, const float stringTnsn) {
        for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
            if (!deeltje.fixed && deeltje.simulable) {
                calculateForcesAndAccelerations(deeltje, springCsnt, springDmper, stringTnsn);
            }
        }
    }

    void updateVelocities(ParticleSystem* deeltjessysteem, const float timestep) {
        const float timeStepDiv2 = 0.5f * timeStep;
        for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
            if (!deeltje.fixed && deeltje.simulable) {
                deeltje.velocity = deeltje.velocity
                    + (deeltje.oldAcceleration + deeltje.newAcceleration) * timeStepDiv2;
            }
        }
    }
};

class SimulatorFPUT {
public:

    const float springConstant;
    const float springDamping;
    const float stringTension;
    const float alpha;
    const float timeStep;

    SimulatorFPUT(float SpringConst, float SpringDamp, float StringTension, float Alpha, float TimeStep) :
        springConstant(SpringConst),
        springDamping(SpringDamp),
        stringTension(StringTension),
        alpha(Alpha),
        timeStep(TimeStep)
    {};

    void verletSolver(ParticleSystem* ps) {
        updatePositions(ps, timeStep);
        updateForcesAndAccelerations(ps, springConstant, springDamping, stringTension);
        updateVelocities(ps, timeStep);
    }

private:

    float distance = 0.0f;
    float offset = 0.0f;
    Vec2 delta_v = Vec2(0.0f, 0.0f);
    Vec2 direction = Vec2(0.0f, 0.0f);

    void calculateForcesAndAccelerations(waterDeeltje& eneDeeltje, const float springCst, const float springDmp, const float stringTns) {
        eneDeeltje.force = Vec2(0.0f, 0.0f); // set force of previous timestep to zero
        for (int i = 0; i < eneDeeltje.neighbours.size(); i++) {
            distance = eneDeeltje.position.DistanceTo(eneDeeltje.neighbours[i]->position); // REMEMBER the array neighbnours contains pointers!
            direction = eneDeeltje.position.DirectionTo(eneDeeltje.neighbours[i]->position);
            offset = eneDeeltje.offsets[i] / stringTns;
            delta_v = eneDeeltje.velocity - eneDeeltje.neighbours[i]->velocity;
            eneDeeltje.force = eneDeeltje.force
                - ((direction * (distance - offset))
                    - direction * (distance - offset)
                    * alpha * (distance - offset)) * springCst
                - direction * springDmp * (delta_v.Dot(direction)); // last term is damping
            eneDeeltje.newAcceleration = eneDeeltje.force / eneDeeltje.mass;
        }
    }

    void updatePositions(ParticleSystem* deeltjessysteem, const float timestep) {
        const float timeStep2Div2 = 0.5f * timeStep * timeStep;
        for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
            if (deeltje.simulable) {
                deeltje.oldAcceleration = deeltje.newAcceleration; // nieuw wordt oud, nu zijn oud en nieuw gelijk
                deeltje.position = deeltje.position
                    + deeltje.velocity * timestep
                    + deeltje.newAcceleration * timeStep2Div2;
            }
        }
    }

    void updateForcesAndAccelerations(ParticleSystem* deeltjessysteem, const float springCsnt, const float springDmper, const float stringTnsn) {
        for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
            if (deeltje.simulable) {
                calculateForcesAndAccelerations(deeltje, springCsnt, springDmper, stringTnsn);
            }
        }
    }

    void updateVelocities(ParticleSystem* deeltjessysteem, const float timestep) {
        const float timeStepDiv2 = 0.5f * timeStep;
        for (auto& deeltje : deeltjessysteem->WaterDeeltjes) {
            if (deeltje.simulable) {
                deeltje.velocity = deeltje.velocity
                    + (deeltje.oldAcceleration + deeltje.newAcceleration) * timeStepDiv2;
            }
        }
    }
};

class ParticleSys1D {
public:
    std::vector<float> positionx = std::vector<float>();
    std::vector<float> positiony = std::vector<float>();
    std::vector<float> velocity = std::vector<float>();
    std::vector<float> oldAcc = std::vector<float>();
    std::vector<float> newAcc = std::vector<float>();
    std::vector<float> force = std::vector<float>();
    std::vector<uint8_t> simulable;
    float mass = 1.0f;

    ParticleSys1D(const int N_Particles) {

        for (int i = 0; i < N_Particles; i++) {
            positionx.emplace_back((float)i); // EMPLACE BACK
            positiony.emplace_back(0.0f); // EMPLACE BACK
            newAcc.emplace_back(0.0f);
            oldAcc.emplace_back(0.0f);
            force.emplace_back(0.0f);
            if (i == 0 || i == N_Particles - 1) {
                velocity.emplace_back(0.0f);
                simulable.emplace_back(false);
            }
            else {
                velocity.emplace_back(1.0f);
                simulable.emplace_back(true);
            }
        }
    }
};

class Simulator1D {
public:
    const float springConstant;
    const float alpha;
    const float timeStep;
    const float timeStepDiv2 = 0.5f * timeStep;
    const float timeStep2Div2 = 0.5f * timeStep * timeStep;

    Simulator1D(const float SpringConst, const float Alpha, const float TimeStep) :
        springConstant(SpringConst),
        alpha(Alpha),
        timeStep(TimeStep)
    {};

    void verletSolver(ParticleSys1D* ps) {
        updatePositions(ps);
        updateForcesAndAccelerations(ps);
        updateVelocities(ps);
    }

private:
    float a = 0.0f;
    float b = 0.0f;


    void updatePositions(ParticleSys1D* ds) {
        for (int i = 0; i < ds->positionx.size(); ++i) {
            if (ds->simulable[i]) {
                ds->oldAcc[i] = ds->newAcc[i]; // nieuw wordt oud, nu zijn oud en nieuw gelijk
                ds->positiony[i] = ds->positiony[i]
                    + ds->velocity[i] * timeStep
                    + ds->newAcc[i] * timeStep2Div2;
            }
        }
    }

    void updateForcesAndAccelerations(ParticleSys1D* ds) {
        for (int i = 1; i < ds->positionx.size() - 1; ++i) {
            // calculate the distances
            a = ds->positiony[i] - ds->positiony[i - 1];
            b = ds->positiony[i] - ds->positiony[i + 1];

            // calculate the SPRING FORCE for both neighbours:
            // left neighbour
            ds->force[i] = ds->force[i] - springConstant * a * (1 + alpha * a);

            // right neighbour:
            ds->force[i] = ds->force[i] - springConstant * b * (1 + alpha * b);

            // niet vergeten deze weer weg te doen:
            ds->newAcc[i] = ds->force[i];
            ds->force[i] = 0.0f; // reset forces
        }
    }

    void updateVelocities(ParticleSys1D* ds) {
        for (int i = 0; i < ds->positionx.size(); ++i) {
            if (ds->simulable[i]) {
                ds->velocity[i] = ds->velocity[i]
                    + (ds->oldAcc[i] + ds->newAcc[i]) * timeStepDiv2;
            }
        }
    }
};