#include "cloth_simulator.hpp"

RectClothSimulator::
RectClothSimulator(
        RectCloth *cloth,
        float totalMass,
        float stiffnessReference,
        float airResistanceCoefficient,
        const glm::vec3& gravity) : cloth(cloth), airResistanceCoefficient(airResistanceCoefficient), gravity(gravity) {
    // Initialize particles, then springs according to the given cloth
    createMassParticles(totalMass);
    createSprings(stiffnessReference);
}

void RectClothSimulator::
createMassParticles(float totalMass) {
    // Create mass particles based on given cloth.
    particles.resize(cloth->nw * cloth->nh);
    for (unsigned int ih = 0; ih < cloth->nh; ih++) {
        for (unsigned int iw = 0; iw < cloth->nw; iw++) {
            MassParticle particle;
            particle.position = cloth->getPosition(iw, ih);

            // TODO: Initialize other mass properties.
            //  Use 'cloth->...' to access cloth properties.
            particle.mass = totalMass / (cloth->nw * cloth->nh);;                // Assign computed mass
            particle.velocity = glm::vec3(0.0f);         // Initial velocity is zero (assuming no initial motion)
            particle.acceleration = glm::vec3(0.0f);     // Initial acceleration is zero
            particle.force = glm::vec3(0.0f);
            particle.previousPosition = particle.position;

            particles[cloth->idxFromCoord(iw, ih)] = particle;
        }
    }
}

void RectClothSimulator::
createSprings(float stiffnessReference) {
    // First clear all springs
    springs.clear();

    // TODO: Create springs connecting mass particles.
    //  You may find 'cloth->idxFromCoord(...)' useful.
    //  You can store springs into the member variable 'springs' which is a std::vector.
    //  You may want to modify mass particles too.
    int numParticlesX = cloth->nw;
    int numParticlesY = cloth->nh;  

    for (unsigned int ih = 0; ih < numParticlesY; ih++) {
        for (unsigned int iw = 0; iw < numParticlesX; iw++) {
            unsigned int currentIndex = cloth->idxFromCoord(iw, ih);

            if (iw < numParticlesX - 1) {
                unsigned int rightIndex = cloth->idxFromCoord(iw + 1, ih);
                float restLength = glm::length(particles[rightIndex].position - particles[currentIndex].position);
                springs.push_back(Spring(currentIndex, rightIndex, stiffnessReference, restLength));

                particles[currentIndex].connectedSpringEndIndices.push_back(rightIndex);
              //  particles[currentIndex].endsti.push_back(stiffnessReference);
                particles[rightIndex].connectedSpringEndIndices.push_back(currentIndex);
            }

            if (ih < numParticlesY - 1) {
                unsigned int downIndex = cloth->idxFromCoord(iw, ih + 1);
                float restLength = glm::length(particles[downIndex].position - particles[currentIndex].position);
                springs.push_back(Spring(currentIndex, downIndex, stiffnessReference, restLength));

                particles[currentIndex].connectedSpringEndIndices.push_back(downIndex);
              //  particles[currentIndex].endsti.push_back(stiffnessReference);
                particles[downIndex].connectedSpringEndIndices.push_back(currentIndex);
            }

            if (iw < numParticlesX - 1 && ih < numParticlesY - 1) {
                unsigned int diagonalIndex = cloth->idxFromCoord(iw + 1, ih + 1);
                float restLength = glm::length(particles[diagonalIndex].position - particles[currentIndex].position);
                springs.push_back(Spring(currentIndex, diagonalIndex, 0.5f*stiffnessReference, restLength));

                particles[currentIndex].connectedSpringEndIndices.push_back(diagonalIndex);
             //   particles[currentIndex].endsti.push_back(0.5f*stiffnessReference);
                particles[diagonalIndex].connectedSpringEndIndices.push_back(currentIndex);
            }

            if (iw < numParticlesX - 1 && ih > 0) {
                unsigned int diagonalUpIndex = cloth->idxFromCoord(iw + 1, ih - 1);
                float restLength = glm::length(particles[diagonalUpIndex].position - particles[currentIndex].position);
                springs.push_back(Spring(currentIndex, diagonalUpIndex, 0.5f*stiffnessReference, restLength));

                particles[currentIndex].connectedSpringEndIndices.push_back(diagonalUpIndex);
               // particles[currentIndex].endsti.push_back(0.5f*stiffnessReference);
                particles[diagonalUpIndex].connectedSpringEndIndices.push_back(currentIndex);
            }
            if (iw > 0) {
                unsigned int leftIndex = cloth->idxFromCoord(iw - 1, ih);
                float restLength = glm::length(particles[leftIndex].position - particles[currentIndex].position);
                springs.push_back(Spring(currentIndex, leftIndex, stiffnessReference, restLength));

                particles[currentIndex].connectedSpringEndIndices.push_back(leftIndex);
              //  particles[currentIndex].endsti.push_back(stiffnessReference);
                particles[leftIndex].connectedSpringEndIndices.push_back(currentIndex);
            }

            if (ih > 0) {
                unsigned int upIndex = cloth->idxFromCoord(iw, ih - 1);
                float restLength = glm::length(particles[upIndex].position - particles[currentIndex].position);
                springs.push_back(Spring(currentIndex, upIndex, stiffnessReference, restLength));

                particles[currentIndex].connectedSpringEndIndices.push_back(upIndex);
                //particles[currentIndex].endsti.push_back(stiffnessReference);
                particles[upIndex].connectedSpringEndIndices.push_back(currentIndex);
            }
            if (iw > 0 && ih < numParticlesY - 1) {
                unsigned int diagonalDownLeftIndex = cloth->idxFromCoord(iw - 1, ih + 1);
                float restLength = glm::length(particles[diagonalDownLeftIndex].position - particles[currentIndex].position);
                springs.push_back(Spring(currentIndex, diagonalDownLeftIndex,0.5f* stiffnessReference, restLength));
                //particles[currentIndex].endsti.push_back(0.5f*stiffnessReference);
                particles[currentIndex].connectedSpringEndIndices.push_back(diagonalDownLeftIndex);
                particles[diagonalDownLeftIndex].connectedSpringEndIndices.push_back(currentIndex);
            }

           
            if (iw > 0 && ih > 0) {
                unsigned int diagonalUpLeftIndex = cloth->idxFromCoord(iw - 1, ih - 1);
                float restLength = glm::length(particles[diagonalUpLeftIndex].position - particles[currentIndex].position);
                springs.push_back(Spring(currentIndex, diagonalUpLeftIndex, 0.5f*stiffnessReference, restLength));

                particles[currentIndex].connectedSpringEndIndices.push_back(diagonalUpLeftIndex);
                //particles[currentIndex].endsti.push_back(0.5f*stiffnessReference);
                particles[diagonalUpLeftIndex].connectedSpringEndIndices.push_back(currentIndex);
            }
        }
    }

}

void RectClothSimulator::step(float timeStep) {
    std::vector<glm::vec3> force_par;
    glm::vec3 wind(0, 0, 0); 
    //glm::vec3 wind(0.01f, 0, 0); 

    force_par.resize(particles.size(), glm::vec3(0.0f, 0.0f, 0.0f));

    for (unsigned int i = 0; i < springs.size(); i++) {
        unsigned int fromID=springs[i].fromMassIndex;
        unsigned int toID=springs[i].toMassIndex;
        glm::vec3 direction=particles[toID].position - particles[fromID].position;
        glm::vec3 springDirection = glm::normalize(direction);
        float springLength=glm::distance(particles[toID].position ,particles[fromID].position);
        float springForceMagnitude = springs[i].stiffness * (springLength - springs[i].restLength);
        glm::vec3 springForce = springForceMagnitude * springDirection;
        force_par[fromID]+=springForce ;
    }

    for (unsigned int i = 0; i < particles.size(); i++) {
        glm::vec3 gravityForce = particles[i].mass * gravity;
        force_par[i]+=gravityForce;
        float a=0.0001;
        particles[i].velocity=(particles[i].position-particles[i].previousPosition)/timeStep;
        float v=a*glm::distance(particles[i].velocity,glm::vec3(0,0,0));
        force_par[i]-=particles[i].velocity*v; 
        force_par[i]+=wind;
    }

    for (unsigned int i = 0; i < particles.size(); i++) {
        glm::vec3 acceleration = force_par[i] / particles[i].mass;
        particles[i].acceleration = acceleration;
    }


    for (unsigned int i = 0; i < particles.size(); i++) {
        if (i == cloth->idxFromCoord(0, cloth->nh-1)|| i == cloth->idxFromCoord(cloth->nw - 1, cloth->nh-1)) {
            particles[i].acceleration=glm::vec3(0,0,0);
            continue;  
        }
        glm::vec3 air = particles[i].acceleration * timeStep ;
        air=air* timeStep;
        glm::vec3 newPosition = 2.0f * particles[i].position - particles[i].previousPosition +air;

        particles[i].previousPosition = particles[i].position;


        particles[i].position = newPosition;

    }
    updateCloth();
}

void RectClothSimulator::
updateCloth() {
    for (unsigned int i = 0u; i < cloth->nw * cloth->nh; i++)
    {
        cloth->setPosition(i, particles[i].position);
    }
}