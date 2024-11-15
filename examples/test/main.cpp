#include "nik3dsim.h"

using namespace nik3dsim;

int main() {
    // Create simulator with custom timestep and substeps for stability
    RigidBodySimulator sim;
    simulator_init(
        &sim, 
        vec3_create(0, 0, 0),  // No gravity for pure rotation demo
        0.016f,                // 16ms timestep (60Hz)
        1                      // 1 position iteration (no constraints)
    );
    
    // Create a T-shaped body by setting appropriate inertia values
    RigidBody body;
    
    // Make a box with dimensions that create distinct principal moments of inertia
    // The T shape will be approximated by setting appropriate inertia values
    // We'll create an effective T shape with:
    // - Long bar: 2 units wide, 0.2 units tall, 0.2 units deep
    // - Vertical bar: 0.2 units wide, 1 unit tall, 0.2 units deep
    Vec3 size = vec3_create(1.0f, 1.0f, 1.0f);  // Base size (will be modified by inertia)
    
    // Initialize the body at the origin with no initial rotation
    rigidbody_init(
        &body,
        BODY_BOX,
        size,
        1.0f,  // Density of 1 for simplicity
        vec3_create(0, 0, 0),  // At origin
        vec3_create(0, 0, 0)   // No initial rotation
    );
    
    // Manually set inertia to match T shape
    // These values create the characteristic intermediate axis instability
    // Ix < Iy < Iz (intermediate axis is y)
    float Ix = 1.0f;    // Smallest (about vertical bar)
    float Iy = 4.0f;    // Intermediate (about horizontal bar)
    float Iz = 5.0f;    // Largest (about depth)
    
    body.invInertia = vec3_create(1.0f/Ix, 1.0f/Iy, 1.0f/Iz);
    
    // Set initial angular velocity
    // Primarily around y-axis (intermediate) with small perturbation
    body.omega = vec3_create(
        0.1f,    // Small perturbation in x
        10.0f,   // Main rotation around y
        0.1f     // Small perturbation in z
    );
    
    // Add some damping to prevent perpetual motion
    body.damping = 0;
    
    // Add body to simulator
    sim.rigidBodies[sim.rigidBodyCount++] = body;
    
    // Run simulation for 1000 frames (about 16 seconds)
    for(int frame = 0; frame < 10000; frame++) {
        simulator_simulate(&sim);
        print_simulation_state_parse(&sim);
    }
    
    return 0;
}