# Coupled particles

This codebase is can be used to simulate a coupled system of particles:

- particles are connected by springs
- a neighbourlist indicates which other particles the particle is connected to.
- I made a math library for 2D vector functions (math2D.h), which includes operator overloads.
- there is a class called waterDeeltje (dutch for waterparticle), the system of waterparticles (ParticleSystem), and for the simulation.
- right now the default configuration is a string of N particles, where the outer particles are fixed.
- the code is written in anticipation for interactive use as a physics engine, hence the 'simulable' boolean quantity.
- the main.cpp file shows an example of how this simulator can be used.

[An example of the use of this library in an Android app can be found here](https://www.youtube.com/watch?v=Ve-RVZ7UdLM)


have fun!
