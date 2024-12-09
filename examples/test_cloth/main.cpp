#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>

using namespace nik3dsim;

int main() {
    nikModel m;
    nikData d;
    niknum gravity[3] = {0, 0, -9.81};
    simulator_init(&m, gravity, 0.001f, 2);  // More iterations for cloth stability

    m.damping = 0.5f;  // Increased damping for cloth behavior
    
    // Parameters for cloth mesh
    const int width = 20;   // Number of particles in width
    const int height = 20;  // Number of particles in height
    const float spacing = 0.1f;  // Space between particles
    const float radius = 0.1f;  // Sphere radius

    // Create static anchor points (two corners)
    RigidBodyModel anchor1model, anchor2model;
    RigidBodyData anchor1data, anchor2data;
    niknum anchorSize[3] = {0.05f, 0.1f, 0.1f};
    niknum anchor1pos[3] = {-spacing * (width-1)/2, 0, 2};
    niknum anchor2pos[3] = {spacing * (width-1)/2, 0, 2};
    niknum noRotation[3] = {0, 0, 0};
    
    rigidbody_init(&anchor1model, &anchor1data, BODY_SPHERE, anchorSize, 1.0f, anchor1pos, noRotation);
    rigidbody_init(&anchor2model, &anchor2data, BODY_SPHERE, anchorSize, 1.0f, anchor2pos, noRotation);
    
    // Make anchors static
    anchor1model.invMass = 0.0f;
    anchor2model.invMass = 0.0f;
    for(int i = 0; i < 3; i++) {
        anchor1model.invInertia[i] = 0.0f;
        anchor2model.invInertia[i] = 0.0f;
    }

    // Add anchors to simulation
    int anchor1_idx = m.rigidBodyCount++;
    int anchor2_idx = m.rigidBodyCount++;
    m.bodies[anchor1_idx] = anchor1model;
    m.bodies[anchor2_idx] = anchor2model;
    d.bodies[anchor1_idx] = anchor1data;
    d.bodies[anchor2_idx] = anchor2data;

    // Add a box
    StaticBodyModel body1model;
    niknum size1[3] = {0.5f, 0.3f, 0.3f};     // Unit cube
    niknum pos1[3] = {0.3, 0, 0.5};           // Positioned left of origin
    niknum angles1[3] = {0, 0, 0};            // No initial rotation
    static_init(&body1model, BODY_SPHERE, size1, pos1, angles1);
    body1model.conaffinity = 1;
    body1model.contype = 1;

    int bidx = m.staticBodyCount++;
    m.staticBodies[bidx] = body1model;

    // Create cloth particles
    int particleIndices[width][height];
    for(int i = 0; i < width; i++) {
        for(int j = 0; j < height; j++) {
            RigidBodyModel sphereModel;
            RigidBodyData sphereData;
            niknum sphereSize[3] = {radius, radius, radius};
            niknum pos[3] = {
                spacing * (i - (width-1)/2.0f),
                spacing * j,
                2.0f
            };
            
            rigidbody_init(&sphereModel, &sphereData, BODY_SPHERE, sphereSize, 0.1f, pos, noRotation);

            sphereModel.contype = 0;
            sphereModel.conaffinity = 1;
            
            particleIndices[i][j] = m.rigidBodyCount++;
            m.bodies[particleIndices[i][j]] = sphereModel;
            d.bodies[particleIndices[i][j]] = sphereData;
        }
    }

    // Create constraints between particles
    niknum localPos[3] = {0, 0, 0};
    
    // Horizontal and vertical constraints
    for(int i = 0; i < width; i++) {
        for(int j = 0; j < height; j++) {
            if(i < width-1) {  // Horizontal connections
                DistanceConstraint constraint;
                constraint.compliance = 0.00001f;  // Slight elasticity
                constraint.b0 = particleIndices[i][j];
                constraint.b1 = particleIndices[i+1][j];
                vec3_copy(constraint.r0, localPos);
                vec3_copy(constraint.r1, localPos);
                constraint.distance = spacing;
                m.positionalConstraints[m.positionalConstraintCount++] = constraint;
            }
            
            if(j < height-1) {  // Vertical connections
                DistanceConstraint constraint;
                constraint.compliance = 0.00001f;
                constraint.b0 = particleIndices[i][j];
                constraint.b1 = particleIndices[i][j+1];
                vec3_copy(constraint.r0, localPos);
                vec3_copy(constraint.r1, localPos);
                constraint.distance = spacing;
                m.positionalConstraints[m.positionalConstraintCount++] = constraint;
            }
        }
    }

    // Connect top corners to anchors
    DistanceConstraint leftAnchorConstraint;
    leftAnchorConstraint.compliance = 0.0f;
    leftAnchorConstraint.b0 = anchor1_idx;
    leftAnchorConstraint.b1 = particleIndices[0][height-1];
    vec3_copy(leftAnchorConstraint.r0, localPos);
    vec3_copy(leftAnchorConstraint.r1, localPos);
    leftAnchorConstraint.distance = 0.001f;
    m.positionalConstraints[m.positionalConstraintCount++] = leftAnchorConstraint;

    DistanceConstraint rightAnchorConstraint;
    rightAnchorConstraint.compliance = 0.0f;
    rightAnchorConstraint.b0 = anchor2_idx;
    rightAnchorConstraint.b1 = particleIndices[width-1][height-1];
    vec3_copy(rightAnchorConstraint.r0, localPos);
    vec3_copy(rightAnchorConstraint.r1, localPos);
    rightAnchorConstraint.distance = 0.001f;
    m.positionalConstraints[m.positionalConstraintCount++] = rightAnchorConstraint;

    // Initialize renderer
    Renderer renderer;
    if (!renderer_init(&renderer, 2000, 1500)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }

    printf("pos constraints: %ld\n", m.positionalConstraintCount);
    
    // Set up camera
    Camera camera;
    niknum cam_pos[3] = {5.0f, 5.0f, 3.0f};
    niknum cam_target[3] = {0.0f, 0.0f, 2.0f};
    niknum cam_up[3] = {0.0f, 0.0f, 1.0f};
    
    for(int i = 0; i < 3; i++) {
        camera.position[i] = cam_pos[i];
        camera.target[i] = cam_target[i];
        camera.up[i] = cam_up[i];
    }
    
    camera.fov = 60.0f;
    camera.aspectRatio = 2000.0f / 1500.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);
    
    // Initialize mouse state
    MouseState mouseState = {
        0, 0, false, false,
        5.0f,  // Initial distance
        0.0f,  // Initial azimuth
        30.0f  // Initial elevation
    };
    
    // Main loop
    bool running = true;
    SDL_Event event;
    Uint32 lastTime = SDL_GetTicks();
    
    while (running) {
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    running = false;
                    break;
                case SDL_KEYDOWN:
                    if (event.key.keysym.sym == SDLK_ESCAPE) {
                        running = false;
                    }
                    break;
                case SDL_WINDOWEVENT:
                    if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                        renderer_resize(&renderer, event.window.data1, event.window.data2);
                    }
                    break;
                case SDL_MOUSEBUTTONDOWN:
                case SDL_MOUSEBUTTONUP:
                case SDL_MOUSEMOTION:
                case SDL_MOUSEWHEEL:
                    handle_mouse_events(event, renderer.camera, mouseState);
                    renderer_set_camera(&renderer, renderer.camera);
                    break;
            }
        }
        
        Uint32 currentTime = SDL_GetTicks();
        float deltaTime = (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;
        
        static float accumulator = 0.0f;
        accumulator += deltaTime;
        while (accumulator >= m.dt) {
            simulator_step(&m, &d);
            accumulator -= m.dt;
        }
        
        renderer_draw_simulation(&renderer, &m, &d);
        SDL_Delay(1);
    }
    
    renderer_cleanup(&renderer);
    return 0;
}