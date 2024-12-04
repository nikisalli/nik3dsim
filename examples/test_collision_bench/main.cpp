#include "nik3dsim.h"
#include <chrono>

using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and iterations
    nikModel m;
    nikData d;
    niknum gravity[3] = {0, 0, -1};
    simulator_init(
        &m,
        gravity,
        0.01f,      // Small timestep for stability
        1          // More iterations for constraint stability
    );

    m.damping = 0.1f;
    
    // Create sliding box
    RigidBodyModel body1model;
    RigidBodyData body1data;
    niknum size1[3] = {0.5f, 0.5f, 0.5f};
    niknum pos1[3] = {1, 0, 0.5f};
    niknum angles1[3] = {0, M_PI / 2, 0};
    body1model.conaffinity = 1;
    body1model.contype = 1;
    rigidbody_init(
        &body1model,
        &body1data,
        BODY_BOX,
        size1,  
        1.0f,   // Density
        pos1,   
        angles1 
    );
    body1model.contactCompliance = 0.001f;
    body1model.frictionCoef = 1000.0f;

    // Create second sliding box
    RigidBodyModel body3model;
    RigidBodyData body3data;
    niknum size3[3] = {0.5f, 0.5f, 0.5f};
    niknum pos3[3] = {1, 3, 0.5f};
    niknum angles3[3] = {0, M_PI / 2, 0};
    body3model.conaffinity = 1;
    body3model.contype = 1;
    rigidbody_init(
        &body3model,
        &body3data,
        BODY_BOX,
        size3,  
        1.0f,   // Density
        pos3,   
        angles3 
    );
    body3model.contactCompliance = 0.001f;
    body3model.frictionCoef = 3.0f;

    // Create inclined plane
    StaticBodyModel body2model;
    body2model.conaffinity = 1;
    body2model.contype = 1;
    niknum size2[3] = {0.0f, 0.0f, 0.0f};
    niknum pos2[3] = {2, 0, 0};
    niknum angles2[3] = {0, -M_PI * 0.02, 0};  // Slight incline
    static_init(
        &body2model,
        BODY_PLANE,
        size2,
        pos2,
        angles2
    );
    body2model.contactCompliance = 0.001f;

    // Add bodies to simulator
    int body1_idx = m.rigidBodyCount++;
    m.bodies[body1_idx] = body1model;
    d.bodies[body1_idx] = body1data;

    int body3_idx = m.rigidBodyCount++;
    m.bodies[body3_idx] = body3model;
    d.bodies[body3_idx] = body3data;

    int body2_idx = m.staticBodyCount++;
    m.staticBodies[body2_idx] = body2model;
    
    // Main loop - run simulation and measure performance
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 5e6; i++) {
        simulator_step(&m, &d);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    printf("%.0f steps/s\n", 5e12 / duration.count());

    return 0;
}