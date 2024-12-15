#include "nik3dsim.h"
#include <SDL2/SDL.h>
#include <SDL_stdinc.h>
#include <cmath>

using namespace nik3dsim;

int main() {
    nikModel m;
    nikData d;
    simulator_init(&m, (float[]){0, 0, -9.81}, 0.005f, 2, 0.3f);

    const int nodesPerSide = 5;
    const float spacing = 0.2f;
    const float radius = 0.1f;
    const float stiffness = 0.2f;

    int particleIndices[nodesPerSide][nodesPerSide][nodesPerSide];
    for(int x = 0; x < nodesPerSide; x++) {
        for(int y = 0; y < nodesPerSide; y++) {
            for(int z = 0; z < nodesPerSide; z++) {
                float pos[3] = {
                    spacing * (x - (nodesPerSide-1)/2.0f),
                    spacing * (y - (nodesPerSide-1)/2.0f),
                    spacing * (z + 2)
                };
                particleIndices[x][y][z] = m.rigidBodyCount;
                add_rigidbody(&m, BODY_SPHERE, (float[]){radius, radius, radius}, 1.0f, pos, (float[]){0, 0, 0}, 0.0f, 0.0f, 1, 0);
            }
        }
    }

    float localPos[3] = {0, 0, 0};
    
    auto addConstraint = [&](int idx1, int idx2) {
        float pos1[3], pos2[3], dist[3];
        vec3_copy(pos1, m.bodies[idx1].initpos);
        vec3_copy(pos2, m.bodies[idx2].initpos);
        dist[0] = pos2[0] - pos1[0];
        dist[1] = pos2[1] - pos1[1];
        dist[2] = pos2[2] - pos1[2];
        float distance = sqrt(dist[0]*dist[0] + dist[1]*dist[1] + dist[2]*dist[2]);
        
        add_distance_constraint(&m, idx1, idx2, localPos, localPos, stiffness, distance);
    };

    for(int x = 0; x < nodesPerSide; x++) {
        for(int y = 0; y < nodesPerSide; y++) {
            for(int z = 0; z < nodesPerSide; z++) {
                int current = particleIndices[x][y][z];
                
                if(x < nodesPerSide-1)
                    addConstraint(current, particleIndices[x+1][y][z]);
                if(y < nodesPerSide-1)
                    addConstraint(current, particleIndices[x][y+1][z]);
                if(z < nodesPerSide-1)
                    addConstraint(current, particleIndices[x][y][z+1]);
                
                if(x < nodesPerSide-1 && y < nodesPerSide-1) {
                    addConstraint(current, particleIndices[x+1][y+1][z]);
                    addConstraint(particleIndices[x+1][y][z], particleIndices[x][y+1][z]);
                }
                
                if(x < nodesPerSide-1 && z < nodesPerSide-1) {
                    addConstraint(current, particleIndices[x+1][y][z+1]);
                    addConstraint(particleIndices[x+1][y][z], particleIndices[x][y][z+1]);
                }
                
                if(y < nodesPerSide-1 && z < nodesPerSide-1) {
                    addConstraint(current, particleIndices[x][y+1][z+1]);
                    addConstraint(particleIndices[x][y+1][z], particleIndices[x][y][z+1]);
                }
                
                if(x < nodesPerSide-1 && y < nodesPerSide-1 && z < nodesPerSide-1) {
                    addConstraint(current, particleIndices[x+1][y+1][z+1]);
                    addConstraint(particleIndices[x+1][y][z], particleIndices[x][y+1][z+1]);
                    addConstraint(particleIndices[x][y+1][z], particleIndices[x+1][y][z+1]);
                    addConstraint(particleIndices[x+1][y+1][z], particleIndices[x][y][z+1]);
                }
            }
        }
    }

    printf("%ld particles, %ld constraints\n", m.rigidBodyCount, m.positionalConstraintCount);

    add_staticbody(&m, BODY_BOX, (float[]){2.0f, 2.0f, 0.1f}, (float[]){0.0f, 0.0f, -1.0f}, (float[]){0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 1, 1);

    data_init(&m, &d);

    Renderer renderer;
    if (!renderer_init(&renderer, 1024, 768)) {
        printf("Failed to initialize renderer\n");
        return 1;
    }

    Camera camera;
    vec3_copy(camera.target, (float[]){0.0f, 0.0f, 1.0f});
    vec3_copy(camera.up, (float[]){0.0f, 0.0f, 1.0f});
    camera.fov = 60.0f;
    camera.aspectRatio = 1024.0f / 768.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 100.0f;
    renderer_set_camera(&renderer, camera);

    MouseState mouseState = {0, 0, false, false, 5.0f, 0.0f, 30.0f};
    update_camera_from_mouse_state(mouseState, renderer.camera);
    renderer_set_camera(&renderer, renderer.camera);

    bool running = true;
    SDL_Event event;
    Uint32 lastTime = SDL_GetTicks();
    float accumulator = 0.0f;

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
                    if (event.key.keysym.sym == SDLK_SPACE) {
                        for(int x = 0; x < nodesPerSide; x++) {
                            for(int y = 0; y < nodesPerSide; y++) {
                                for(int z = 0; z < nodesPerSide; z++) {
                                    d.bodies[particleIndices[x][y][z]].vel[2] = 2.0f;
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