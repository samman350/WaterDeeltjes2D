#include <stdio.h>
#include <cmath>
#include <iostream>
#include "math2D.h"
#include "waterDeeltje.h"

int main() {
	ParticleSystem* deeltjesSys = new ParticleSystem(4);

	std::cout << deeltjesSys->WaterDeeltjes.size() << std::endl;
}


