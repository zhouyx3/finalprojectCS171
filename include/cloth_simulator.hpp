#pragma once

#include "cloth.hpp"
class RectClothSimulator {
private:
    struct MassParticle {
        glm::vec3 position;
        std::vector<unsigned int> connectedSpringStartIndices;
        std::vector<unsigned int> connectedSpringEndIndices;

        glm::vec3 previousPosition;
        glm::vec3 velocity;         
        glm::vec3 acceleration;   
        glm::vec3 force;         
        float mass;                

        // TODO: define other particle properties here
        
    };

    struct Spring
    {
        unsigned int fromMassIndex;
        unsigned int toMassIndex;

        // TODO: define other spring properties here
        float stiffness;             
        float restLength;         
    };

    RectCloth* cloth;
    std::vector<MassParticle> particles;
    std::vector<Spring> springs;

    // Simulation parameters
    glm::vec3 gravity;
    float airResistanceCoefficient; // Per-particle

public:
    RectClothSimulator(
            RectCloth* cloth,
            float totalMass,
            float stiffnessReference,
            float airResistanceCoefficient,
            const glm::vec3& gravity);
    ~RectClothSimulator() = default;

    void step(float timeStep);

private:
    void createMassParticles(float totalMass);
    void createSprings(float stiffnessReference);
    void updateCloth();
};