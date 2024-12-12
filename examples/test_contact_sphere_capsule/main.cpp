#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>

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
        1           // More iterations for constraint stability
    );

    m.damping = 0.1f;
    
    // Create falling sphere
    RigidBodyModel sphereModel;
    RigidBodyData sphereData;
    niknum sphereSize[3] = {0.5f, 0.0f, 0.0f};  // Radius of 0.5
    niknum spherePos[3] = {0, 0, 10.0f};        // Start above origin
    niknum sphereAngles[3] = {0, 0, 0};         // No initial rotation
    sphereModel.conaffinity = 1;
    sphereModel.contype = 1;
    rigidbody_init(
        &sphereModel,
        &sphereData,
        nik3dsim::BODY_SPHERE,
        sphereSize,  
        1.0f,        // Density
        spherePos,   
        sphereAngles 
    );
    sphereModel.frictionCoef = 0.5f;

    // Create static capsule
    StaticBodyModel capsuleModel;
    capsuleModel.conaffinity = 1;
    capsuleModel.contype = 1;
    niknum capsuleSize[3] = {0.3f, 2.0f, 0.0f};  // Radius 0.3, Height 2.0
    niknum capsulePos[3] = {0.0f, 0.0f, 0.0f};   // At origin
    niknum capsuleAngles[3] = {0, M_PI * 0.25, 0}; // Rotated 45 degrees around Y
    static_init(
        &capsuleModel,
        nik3dsim::BODY_CAPSULE,
        capsuleSize,
        capsulePos,
        capsuleAngles
    );

    // Add bodies to simulator
    int sphereIdx = m.rigidBodyCount++;
    m.bodies[sphereIdx] = sphereModel;
    d.bodies[sphereIdx] = sphereData;

    int capsuleIdx = m.staticBodyCount++;
    m.staticBodies[capsuleIdx] = capsuleModel;
    
    // Initialize renderer
    Renderer renderer;
    if (!renderer_init(&renderer, 2000, 1200)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }
    
    // Set up camera
    Camera camera;
    niknum cam_pos[3] = {0.0f, 15.0f, 0.0f};
    niknum cam_target[3] = {0.0f, 0.0f, 0.0f};
    niknum cam_up[3] = {0.0f, 0.0f, 1.0f};
    
    for(int i = 0; i < 3; i++) {
        camera.position[i] = cam_pos[i];
        camera.target[i] = cam_target[i];
        camera.up[i] = cam_up[i];
    }
    
    camera.fov = 60.0f;
    camera.aspectRatio = 2000.0f / 1200.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);
    
    // Initialize mouse state for camera control
    MouseState mouseState = {
        0,      // lastX
        0,      // lastY
        false,  // leftButtonDown
        false,  // rightButtonDown
        15.0f,  // dist
        0.0f,   // azim
        0.0f    // elev
    };
    
    // Main loop
    bool running = true;
    SDL_Event event;
    Uint32 lastTime = SDL_GetTicks();
    float accumulator = 0.0f;
    
    while (running) {
        // Handle events
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
        
        // Update simulation
        Uint32 currentTime = SDL_GetTicks();
        float deltaTime = (currentTime - lastTime) / 300.0f;
        lastTime = currentTime;
        accumulator += deltaTime;
        
        while (accumulator >= m.dt) {
            simulator_step(&m, &d);
            
            // Visualize contacts
            for (int i = 0; i < d.contactCount; i++) {
                Contact* contact = &d.contacts[i];
                niknum dir[3], length;
                vec3_sub(dir, contact->pos1, contact->pos0);
                length = vec3_normalize(dir, dir);
                // Draw contact normal in red
                renderer_draw_wireframe_arrow(&renderer, contact->pos0, dir, 1.0f, 0.1f, 0.1f, 1.0f, 1.0f, 0.0f);
                
                // Print contact info
                printf("Contact %d - Depth: %.3f\n", i, contact->depth);
                printf("  Pos0: %.3f %.3f %.3f\n", contact->pos0[0], contact->pos0[1], contact->pos0[2]);
                printf("  Pos1: %.3f %.3f %.3f\n", contact->pos1[0], contact->pos1[1], contact->pos1[2]);
            }
            
            accumulator -= m.dt;
        }

        // Render scene
        renderer_draw_simulation(&renderer, &m, &d);
        SDL_Delay(1);
    }
    
    renderer_cleanup(&renderer);
    return 0;
}