#include "nik3dsim.h"

#include <chrono>

using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and iterations
    nikModel m;
    nikData d;
    niknum gravity[3] = {0, 0, -10};
    simulator_init(
        &m,
        gravity,
        0.01f,      // Small timestep for stability
        1          // More iterations for constraint stability
    );

    m.damping = 0.0f;
    
    // Create first cube
    RigidBodyModel body1model;
    RigidBodyData body1data;
    niknum size1[3] = {1.0f, 1.0f, 1.0f};     // Unit cube
    niknum pos1[3] = {0, 0, 0};           // Positioned left of origin
    niknum angles1[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body1model,
        &body1data,
        BODY_BOX,
        size1,  
        1.0f,   // Density
        pos1,   
        angles1 
    );
    
    // Create second cube
    RigidBodyModel body2model;
    RigidBodyData body2data;
    niknum size2[3] = {0.2f, 4.0f, 0.2f};     // Unit cube
    niknum pos2[3] = {0, 2, 0};            // Positioned right of origin
    niknum angles2[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body2model,
        &body2data,
        BODY_BOX,
        size2,
        1.0f,   // Same density
        pos2,
        angles2
    );

    // Create third cube
    RigidBodyModel body3model;
    RigidBodyData body3data;
    niknum size3[3] = {0.2f, 4.0f, 0.2f};     // Unit cube
    niknum pos3[3] = {0, 6, 0};            // Positioned right of origin
    niknum angles3[3] = {0, 0, 0};            // No initial rotation
    rigidbody_init(
        &body3model,
        &body3data,
        BODY_BOX,
        size3,
        1.0f,   // Same density
        pos3,
        angles3
    );
    
    // Fix first body in place
    body1model.invMass = 0.0f;
    body1model.invInertia[0] = 0.0f;
    body1model.invInertia[1] = 0.0f;
    body1model.invInertia[2] = 0.0f;
    
    // Add bodies to simulator
    int body1_idx = m.rigidBodyCount++;
    m.rigidBodies[body1_idx] = body1model;
    d.rigidBodies[body1_idx] = body1data;
    
    int body2_idx = m.rigidBodyCount++;
    m.rigidBodies[body2_idx] = body2model;
    d.rigidBodies[body2_idx] = body2data;

    int body3_idx = m.rigidBodyCount++;
    m.rigidBodies[body3_idx] = body3model;
    d.rigidBodies[body3_idx] = body3data;
    
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
    m.positionalConstraints[m.positionalConstraintCount++] = pos_constraint;

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
    m.hingeConstraints[m.hingeConstraintCount++] = hinge_constraint;

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
    m.positionalConstraints[m.positionalConstraintCount++] = pos_constraint2;

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
    m.hingeConstraints[m.hingeConstraintCount++] = hinge_constraint2;
    
    // Main loop
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 5e6; i++) {
        simulator_simulate(&m, &d);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    printf("%.0f steps/s\n", 5e12 / duration.count());

    return 0;
}