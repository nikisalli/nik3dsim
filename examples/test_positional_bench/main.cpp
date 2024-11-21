#include "nik3dsim.h"

#include <chrono>

using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and iterations
    RigidBodySimulator sim;
    niknum gravity[3] = {0, 0, -10};
    simulator_init(
        &sim,
        gravity,
        0.01f,      // Small timestep for stability
        1          // More iterations for constraint stability
    );
    
    // Create first cube
    RigidBody body1;
    niknum size1[3] = {1.0f, 1.0f, 1.0f};     // Unit cube
    niknum pos1[3] = {0, 0, 0};           // Positioned left of origin
    niknum angles1[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body1,
        BODY_BOX,
        size1,  
        1.0f,   // Density
        pos1,   
        angles1 
    );
    
    // Create second cube
    RigidBody body2;
    niknum size2[3] = {1.0f, 1.0f, 1.0f};     // Unit cube
    niknum pos2[3] = {0, 0, -1};            // Positioned right of origin
    niknum angles2[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body2,
        BODY_BOX,
        size2,
        1.0f,   // Same density
        pos2,
        angles2
    );
    
    // Fix first body in place
    body1.invMass = 0.0f;
    body1.invInertia[0] = 0.0f;
    body1.invInertia[1] = 0.0f;
    body1.invInertia[2] = 0.0f;
    
    // Add bodies to simulator
    int body1_idx = sim.rigidBodyCount++;
    sim.rigidBodies[body1_idx] = body1;
    
    int body2_idx = sim.rigidBodyCount++;
    sim.rigidBodies[body2_idx] = body2;
    
    // Create positional constraint
    DistanceConstraint pos_constraint;
    
    // Set attachment points at the facing corners of the cubes
    niknum local_pos0[3] = {0, 0, 0};   // Right-top-front corner of body1
    niknum local_pos1[3] = {0.5, 0.5, 0.5};  // Left-top-front corner of body2
    
    // Initialize the constraint with some compliance for a slightly soft connection
    pos_constraint.compliance = 0.0f;
    pos_constraint.b0 = body1_idx;
    pos_constraint.b1 = body2_idx;
    vec3_copy(pos_constraint.r0, local_pos0);
    vec3_copy(pos_constraint.r1, local_pos1);
    pos_constraint.distance = 1.0f;
    
    // Add constraint to simulator
    sim.positionalConstraints[sim.positionalConstraintCount++] = pos_constraint;
    
    // Set up camera to view the scene
    
    // Main loop
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 5e6; i++) {
        simulator_simulate(&sim);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    printf("%.0f steps/s\n", 5e12 / duration.count());

    return 0;
}