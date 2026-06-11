#include "particle.h"
#include "..\..\config.h"

Particle::Particle() {

}

// must NOT delete map!
Particle::~Particle() {
    
}

// Map pointer points to the same map
Particle* Particle::Copy() {
    Particle* newParticle = new Particle();

    newParticle->oldScanX = this->oldScanX;
    newParticle->oldScanY = this->oldScanY;
    newParticle->oldScanTheta = this->oldScanTheta;

    newParticle->currScanX = this->currScanX;
    newParticle->currScanY = this->currScanY;
    newParticle->currScanTheta = this->currScanTheta;

    newParticle->x = this->x;
    newParticle->y = this->y;
    newParticle->theta = this->theta;

    newParticle->nudgeX = this->nudgeX;
    newParticle->nudgeY = this->nudgeY;
    newParticle->nudgeTheta = this->nudgeTheta;
    
    newParticle->weight = this->weight;
    newParticle->map = this->map;

    return newParticle;
}

void Particle::UpdateHistory() {
    this->oldScanX = this->currScanX;
    this->oldScanY = this->currScanY;
    this->oldScanTheta = this->currScanTheta;

    this->currScanX = this->x;
    this->currScanY = this->y;
    this->currScanTheta = this->theta;
}
