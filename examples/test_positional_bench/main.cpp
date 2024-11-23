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

    sim.damping = 0.0f;
    
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
    niknum size2[3] = {0.2f, 4.0f, 0.2f};     // Unit cube
    niknum pos2[3] = {0, 2, 0};            // Positioned right of origin
    niknum angles2[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body2,
        BODY_BOX,
        size2,
        1.0f,   // Same density
        pos2,
        angles2
    );

    // Create third cube
    RigidBody body3;
    niknum size3[3] = {0.2f, 4.0f, 0.2f};     // Unit cube
    niknum pos3[3] = {0, 6, 0};            // Positioned right of origin
    niknum angles3[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body3,
        BODY_BOX,
        size3,
        1.0f,   // Same density
        pos3,
        angles3
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

    int body3_idx = sim.rigidBodyCount++;
    sim.rigidBodies[body3_idx] = body3;
    
    // Create positional constraint
    DistanceConstraint pos_constraint;
    pos_constraint.compliance = 0.0f;
    pos_constraint.b0 = body1_idx;
    pos_constraint.b1 = body2_idx;
    pos_constraint.r0[0] = 0.0f;
    pos_constraint.r0[1] = 0.0f;
    pos_constraint.r0[2] = 0.0f;
    pos_constraint.r1[0] = 0.0f;
    pos_constraint.r1[1] = -2.0f;
    pos_constraint.r1[2] = 0.0f;
    pos_constraint.distance = 0.0f;
    sim.positionalConstraints[sim.positionalConstraintCount++] = pos_constraint;

    // Create hinge constraint
    HingeConstraint hinge_constraint;
    hinge_constraint.b0 = body1_idx;
    hinge_constraint.b1 = body2_idx;
    hinge_constraint.a0[0] = 1.0f;
    hinge_constraint.a0[1] = 0.0f;
    hinge_constraint.a0[2] = 0.0f;
    hinge_constraint.a1[0] = 1.0f;
    hinge_constraint.a1[1] = 0.0f;
    hinge_constraint.a1[2] = 0.0f;
    hinge_constraint.compliance = 0.000000001f;
    sim.hingeConstraints[sim.hingeConstraintCount++] = hinge_constraint;

    // Create positional constraint
    DistanceConstraint pos_constraint2;
    pos_constraint2.compliance = 0.0f;
    pos_constraint2.b0 = body2_idx;
    pos_constraint2.b1 = body3_idx;
    pos_constraint2.r0[0] = 0.0f;
    pos_constraint2.r0[1] = 2.0f;
    pos_constraint2.r0[2] = 0.0f;
    pos_constraint2.r1[0] = 0.0f;
    pos_constraint2.r1[1] = -2.0f;
    pos_constraint2.r1[2] = 0.0f;
    pos_constraint2.distance = 0.0f;
    sim.positionalConstraints[sim.positionalConstraintCount++] = pos_constraint2;

    // Create hinge constraint
    HingeConstraint hinge_constraint2;
    hinge_constraint2.b0 = body2_idx;
    hinge_constraint2.b1 = body3_idx;
    hinge_constraint2.a0[0] = 1.0f;
    hinge_constraint2.a0[1] = 0.0f;
    hinge_constraint2.a0[2] = 0.0f;
    hinge_constraint2.a1[0] = 1.0f;
    hinge_constraint2.a1[1] = 0.0f;
    hinge_constraint2.a1[2] = 0.0f;
    hinge_constraint2.compliance = 0.000000001f;
    sim.hingeConstraints[sim.hingeConstraintCount++] = hinge_constraint2;
    
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