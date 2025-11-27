#pragma once

#include <cmath>
#include <iostream>
#include "math2D.h"
#include <vector>

constexpr float M_PI = (float)3.14159265358979323846;

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
    bool simulable; // is the particle motion dictated by forces (true), or static/other (false)?

    waterDeeltje(Vec2 initPos, Vec2 initVel, Vec2 initAcc, float massParticle, bool simulatable) :
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
                    false); // fix outer particles

            }
            else {
                WaterDeeltjes.emplace_back(Vec2((float)i, 0.0f),
                    Vec2(0.0f, 1.0f),
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
        }
    }
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

// ALTERNATIVE MET BENDING STRESS

class waterDeeltjeB {
public:
    Vec2 position;
    Vec2 velocity;
    Vec2 oldAcceleration = Vec2(0.0f, 0.0f);
    Vec2 newAcceleration; // de verschillende definities van acc hebben met solver te maken
    float angle = M_PI; // voor angle bending mode, curvature cost

    Vec2 force = Vec2(0.0f, 0.0f);
    std::vector<waterDeeltjeB*> neighbours = std::vector<waterDeeltjeB*>();
    std::vector<float> offsets = std::vector<float>();

    float mass;
    bool simulable; // is the particle motion dictated by forces (true), or static/other (false)?

    waterDeeltjeB(Vec2 initPos, Vec2 initVel, Vec2 initAcc, float massParticle, bool simulatable) :
        position(initPos), // initializer list
        velocity(initVel),
        newAcceleration(initAcc),
        mass(massParticle),
        simulable(simulatable)
    {}
};

class ParticleSystemB {
public:
    std::vector<Vec2> position = std::vector<Vec2>();
    std::vector<Vec2> velocity = std::vector<Vec2>();
    std::vector<Vec2> oldAcc = std::vector<Vec2>();
    std::vector<Vec2> newAcc = std::vector<Vec2>();
    std::vector<Vec2> force = std::vector<Vec2>();
    std::vector<float> angle = std::vector<float>();
    std::vector<float> offset = std::vector<float>();
    std::vector<uint8_t> simulable;
    float mass = 1.0f;

    ParticleSystemB(const int N_Particles, const float r_cutoff) {

        for (int i = 0; i < N_Particles; i++) {
            position.emplace_back(Vec2((float)i, 0.0f)); // EMPLACE BACK
            newAcc.emplace_back(Vec2(0.0f, 0.0f));
            oldAcc.emplace_back(Vec2(0.0f, 0.0f));
            force.emplace_back(Vec2(0.0f, 0.0f));
            angle.emplace_back(0.0f);
            if (i == 0 || i == N_Particles - 1) {
                velocity.emplace_back(Vec2(0.0f, 0.0f));
                simulable.emplace_back(false);
            }
            else {
                velocity.emplace_back(Vec2(0.0f, 1.0f));
                simulable.emplace_back(true);
            }
        }

        // vul de burenlijst (als afstand kleiner dan cutoff dan buren)
        for (int i = 0; i < N_Particles; i++) {
            for (int j = 0; j < N_Particles; j++) {
                if (i != j && position[i].DistanceTo(position[j]) < r_cutoff) {
                    offset.emplace_back(
                        position[i].DistanceTo(position[j])); // fill offsets list
                }
            }
        }
    }
};

class SimulatorB {
public:

    const float springConstant;
    const float springDamping;
    const float stringTension;
    const float stringStress;
    const float timeStep;
    const float timeStepDiv2 = 0.5f * timeStep;
    const float timeStep2Div2 = 0.5f * timeStep * timeStep;

    SimulatorB(float SpringConst, float SpringDamp, float StringTension, float StringStress, float TimeStep) :
        springConstant(SpringConst),
        springDamping(SpringDamp),
        stringTension(StringTension),
        stringStress(StringStress),
        timeStep(TimeStep)
    {};

    void verletSolver(ParticleSystemB* ps) {
        updatePositions(ps);
        updateForcesAndAccelerations(ps);
        updateVelocities(ps);
    }

private:

    float distance = 0.0f;
    float a = 0.0f; // distance to left neighbour
    float b = 0.0f; // distance to right neighbour
    float c = 0.0f; // distance between neighbours
    float offsetPart = 0.0f;
    Vec2 delta_v = Vec2(0.0f, 0.0f);
    Vec2 direction = Vec2(0.0f, 0.0f);
    std::vector<Vec2> directions = { Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f} };
    std::vector<Vec2> angleForce = { Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f} };

    void calculateForcesAndAccelerations(std::vector<Vec2>& position, 
                                        std::vector<Vec2>& velocity, 
                                        std::vector<Vec2>& force, 
                                        std::vector<Vec2>& accelerationNew,
                                        std::vector<float>& angle,
                                        const std::vector<float>& offsets,
                                        const float mass) {
        
    }

    void updatePositions(ParticleSystemB* deeltjessysteem) {
        for (int i = 0; i < deeltjessysteem->position.size(); ++i) {
            if (deeltjessysteem->simulable[i]) {
                deeltjessysteem->oldAcc[i] = deeltjessysteem->newAcc[i]; // nieuw wordt oud, nu zijn oud en nieuw gelijk
                deeltjessysteem->position[i] = deeltjessysteem->position[i]
                    + deeltjessysteem->velocity[i] * timeStep
                    + deeltjessysteem->newAcc[i] * timeStep2Div2;
            }
        }
    }

    void updateForcesAndAccelerations(ParticleSystemB* ds) {
        // skip the first and last particle, since we need triplets, and outer particles have only one neighbour
        for (int i = 1; i < ds->position.size() - 1; ++i) {
            // calculate the distances
            a = ds->position[i].DistanceTo(ds->position[i - 1]);
            b = ds->position[i].DistanceTo(ds->position[i + 1]);

            // then we determine normalized directional vectors
            directions[0] = ds->position[i].DirectionTo(ds->position[i - 1]);
            directions[1] = ds->position[i].DirectionTo(ds->position[i + 1]);

            offsetPart = ds->offset[i] / stringTension; // slimme manier he, om tension te definieren

            // calculate the SPRING FORCE for both neighbours:
            // left neighbour:
            delta_v = ds->velocity[i] - ds->velocity[i - 1];
            ds->force[i] = ds->force[i] - directions[0] * (a - offsetPart) * springConstant
                - directions[0] * springDamping * (delta_v.Dot(directions[0]));

            // right neighbour:
            delta_v = ds->velocity[i] - ds->velocity[i + 1];
            ds->force[i] = ds->force[i] - directions[1] * (b - offsetPart) * springConstant
                - directions[1] * springDamping * (delta_v.Dot(directions[1]));

            // niet vergeten deze weer weg te doen:
            ds->newAcc[i] = ds->force[i] / ds->mass;
            ds->force[i] = Vec2{ 0.0f, 0.0f }; // reset forces

            // vervolg, nu ANGLE BENDING
            
            /*
            c = ds->position[i - 1].DistanceTo(ds->position[i + 1]); // neighbour-to-neighbour distance (for calculation of theta)

            // from distances we calculate the angle between 3 particles via inverse cosine rule
            ds->angle[i] = std::acos((a * a + b * b - c * c) / (2 * a * b));

            // calculate angle bending forces via distances, directions and angle
            angleForce[0] = Vec2(directions[0].y, -directions[0].x) * (ds->angle[i] - M_PI) * (-stringStress / a); // left particle
            angleForce[2] = Vec2(-directions[1].y, directions[1].x) * (ds->angle[i] - M_PI) * (-stringStress / b); // right particle
            angleForce[1] = (angleForce[0] + angleForce[2]) * (-1.0f); // middle particle is the opposite of the other two forces

            // add the angle forces to the total force:
            ds->force[i - 1] = ds->force[i - 1] + angleForce[0];
            ds->force[i] = ds->force[i] + angleForce[1];
            ds->force[i + 1] = ds->force[i + 1] + angleForce[2];

            // only update the acceleration of the left most particle in the trio, since it has seen all the forces:

            ds->newAcc[i - 1] = ds->force[i - 1] / ds->mass;
            ds->force[i - 1] = Vec2{ 0.0f, 0.0f }; // reset forces

            if (i == ds->position.size() - 2) { // if you reached the last particle, update acc of middle particle (i) too
                ds->newAcc[i] = ds->force[i] / ds->mass;
                ds->force[i] = Vec2{ 0.0f, 0.0f }; // reset forces
            }
            */
        }
        
    }

    void updateVelocities(ParticleSystemB* deeltjessysteem) {
        for (int i = 0; i < deeltjessysteem->position.size(); ++i) {
            if (deeltjessysteem->simulable[i]) {
                deeltjessysteem->velocity[i] = deeltjessysteem->velocity[i]
                    + (deeltjessysteem->oldAcc[i] + deeltjessysteem->newAcc[i]) * timeStepDiv2;
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