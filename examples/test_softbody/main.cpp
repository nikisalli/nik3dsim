#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>
#include <cmath>

using namespace nik3dsim;

int main() {
    nikModel m;
    nikData d;
    niknum gravity[3] = {0, 0, -9.81};
    simulator_init(&m, gravity, 0.0005f, 4);

    m.damping = 0.3f;
    
    // Cube parameters
    const int nodesPerSide = 4;  // Adjustable: number of nodes per cube edge
    const float spacing = 0.2f;
    const float radius = 0.05f;
    const float stiffness = 1.0f;

    // Create particles in a 3D grid
    int particleIndices[nodesPerSide][nodesPerSide][nodesPerSide];
    for(int x = 0; x < nodesPerSide; x++) {
        for(int y = 0; y < nodesPerSide; y++) {
            for(int z = 0; z < nodesPerSide; z++) {
                RigidBodyModel sphereModel;
                RigidBodyData sphereData;
                niknum sphereSize[3] = {radius, radius, radius};
                niknum pos[3] = {
                    spacing * (x - (nodesPerSide-1)/2.0f),
                    spacing * (y - (nodesPerSide-1)/2.0f),
                    spacing * (z + 2)  // Offset in z to start above ground
                };
                niknum noRotation[3] = {0, 0, 0};
                
                rigidbody_init(&sphereModel, &sphereData, BODY_SPHERE, sphereSize, 1.0f, pos, noRotation);

                sphereModel.contype = 1;
                sphereModel.conaffinity = 0;
                sphereModel.frictionCoef = 0;
                
                particleIndices[x][y][z] = m.rigidBodyCount++;
                m.bodies[particleIndices[x][y][z]] = sphereModel;
                d.bodies[particleIndices[x][y][z]] = sphereData;
            }
        }
    }

    // Create constraints between particles
    niknum localPos[3] = {0, 0, 0};
    
    // Function to add a distance constraint between two particles
    auto addConstraint = [&](int idx1, int idx2, float compliance) {
        DistanceConstraint constraint;
        constraint.compliance = compliance;
        constraint.b0 = idx1;
        constraint.b1 = idx2;
        vec3_copy(constraint.r0, localPos);
        vec3_copy(constraint.r1, localPos);
        
        // Calculate actual distance between particles for natural rest length
        niknum pos1[3], pos2[3];
        vec3_copy(pos1, d.bodies[idx1].pos);
        vec3_copy(pos2, d.bodies[idx2].pos);
        niknum dist[3] = {pos2[0]-pos1[0], pos2[1]-pos1[1], pos2[2]-pos1[2]};
        constraint.distance = sqrt(dist[0]*dist[0] + dist[1]*dist[1] + dist[2]*dist[2]);
        
        m.positionalConstraints[m.positionalConstraintCount++] = constraint;
    };

    // Add constraints along cube edges and diagonals
    for(int x = 0; x < nodesPerSide; x++) {
        for(int y = 0; y < nodesPerSide; y++) {
            for(int z = 0; z < nodesPerSide; z++) {
                int current = particleIndices[x][y][z];
                
                // Edge constraints
                if(x < nodesPerSide-1)
                    addConstraint(current, particleIndices[x+1][y][z], stiffness);
                if(y < nodesPerSide-1)
                    addConstraint(current, particleIndices[x][y+1][z], stiffness);
                if(z < nodesPerSide-1)
                    addConstraint(current, particleIndices[x][y][z+1], stiffness);
                
                // Face diagonal constraints
                if(x < nodesPerSide-1 && y < nodesPerSide-1)
                    addConstraint(current, particleIndices[x+1][y+1][z], stiffness);
                if(x < nodesPerSide-1 && z < nodesPerSide-1)
                    addConstraint(current, particleIndices[x+1][y][z+1], stiffness);
                if(y < nodesPerSide-1 && z < nodesPerSide-1)
                    addConstraint(current, particleIndices[x][y+1][z+1], stiffness);
                
                // Body diagonal constraint
                if(x < nodesPerSide-1 && y < nodesPerSide-1 && z < nodesPerSide-1)
                    addConstraint(current, particleIndices[x+1][y+1][z+1], stiffness);
            }
        }
    }

    printf("%ld particles, %ld constraints\n", m.rigidBodyCount, m.positionalConstraintCount);

    // Add ground plane
    StaticBodyModel groundModel;
    niknum groundSize[3] = {2.0f, 2.0f, 0.1f};
    niknum groundPos[3] = {0.0f, 0.0f, -1.0f};
    niknum groundRot[3] = {0.0f, 0.2f, 0.0f};
    static_init(&groundModel, BODY_BOX, groundSize, groundPos, groundRot);

    groundModel.contype = 1;
    groundModel.conaffinity = 1;
    
    int groundIdx = m.staticBodyCount++;
    m.staticBodies[groundIdx] = groundModel;

    // Initialize renderer
    Renderer renderer;
    if (!renderer_init(&renderer, 1024, 768)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }
    
    // Set up camera
    Camera camera;
    niknum cam_pos[3] = {2.0f, 2.0f, 3.0f};
    niknum cam_target[3] = {0.0f, 0.0f, 1.0f};
    niknum cam_up[3] = {0.0f, 0.0f, 1.0f};
    
    for(int i = 0; i < 3; i++) {
        camera.position[i] = cam_pos[i];
        camera.target[i] = cam_target[i];
        camera.up[i] = cam_up[i];
    }
    
    camera.fov = 60.0f;
    camera.aspectRatio = 1024.0f / 768.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);
    
    MouseState mouseState = {0, 0, false, false, 5.0f, 0.0f, 30.0f};
    
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
                    // Add impulse on spacebar
                    if (event.key.keysym.sym == SDLK_SPACE) {
                        for(int x = 0; x < nodesPerSide; x++) {
                            for(int y = 0; y < nodesPerSide; y++) {
                                for(int z = 0; z < nodesPerSide; z++) {
                                    d.bodies[particleIndices[x][y][z]].vel[1] = 2.0f;
                                }
                            }
                        }
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