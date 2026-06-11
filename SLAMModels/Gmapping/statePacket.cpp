#include "statePacket.h"
#include "particle.h"
#include "..\..\config.h"

StatePacket::StatePacket() {

}

StatePacket::~StatePacket() {
    delete this->pointCloud;
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        delete this->particles[i];
    }
    delete[] this->particles;
}

void StatePacket::CopyInfo(Particle** particles) {
    Particle** copiedParticles = new Particle*[GMAPPING_NUM_PARTICLES];

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        copiedParticles[i] = particles[i]->Copy();
    }

    this->particles = copiedParticles;
}
