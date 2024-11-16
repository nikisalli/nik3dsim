#include "render.hpp"
#include <cmath>

namespace nik3dsim {

bool renderer_init(Renderer* renderer, int width, int height) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        return false;
    }
    
    renderer->window = SDL_CreateWindow(
        "3D Rigid Body Simulation",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        width, height,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );
    
    if (!renderer->window) {
        return false;
    }
    
    renderer->sdl_renderer = SDL_CreateRenderer(
        renderer->window, 
        -1, 
        SDL_RENDERER_ACCELERATED
    );
    
    if (!renderer->sdl_renderer) {
        SDL_DestroyWindow(renderer->window);
        return false;
    }
    
    renderer->screenWidth = width;
    renderer->screenHeight = height;
    
    // Setup default camera
    Camera cam;
    cam.position = vec3_create(5.0f, 5.0f, 5.0f);
    cam.target = vec3_create(0.0f, 0.0f, 0.0f);
    cam.up = vec3_create(0.0f, 1.0f, 0.0f);
    cam.fov = 60.0f;
    cam.aspectRatio = (float)width / height;
    cam.nearPlane = 0.1f;
    cam.farPlane = 100.0f;
    
    renderer_set_camera(renderer, cam);
    return true;
}

void renderer_cleanup(Renderer* renderer) {
    if (renderer->sdl_renderer) {
        SDL_DestroyRenderer(renderer->sdl_renderer);
    }
    if (renderer->window) {
        SDL_DestroyWindow(renderer->window);
    }
    SDL_Quit();
}

void renderer_clear(Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer->sdl_renderer);
}

void renderer_present(Renderer* renderer) {
    SDL_RenderPresent(renderer->sdl_renderer);
}

void renderer_set_camera(Renderer* renderer, Camera camera) {
    renderer->camera = camera;
    
    // Calculate view matrix
    Vec3 z = vec3_normalize(vec3_sub(camera.position, camera.target));
    Vec3 x = vec3_normalize(vec3_cross(camera.up, z));
    Vec3 y = vec3_cross(z, x);
    
    // Build view matrix (row-major)
    Vec3 neg_pos = vec3_scale(camera.position, -1.0f);
    renderer->viewMatrix[0] = x.x;  
    renderer->viewMatrix[4] = x.y;  
    renderer->viewMatrix[8] = x.z;  
    renderer->viewMatrix[12] = vec3_dot(x, neg_pos);
    
    renderer->viewMatrix[1] = y.x;  
    renderer->viewMatrix[5] = y.y;  
    renderer->viewMatrix[9] = y.z;  
    renderer->viewMatrix[13] = vec3_dot(y, neg_pos);
    
    renderer->viewMatrix[2] = z.x;  
    renderer->viewMatrix[6] = z.y;  
    renderer->viewMatrix[10] = z.z; 
    renderer->viewMatrix[14] = vec3_dot(z, neg_pos);
    
    renderer->viewMatrix[3] = 0.0f; 
    renderer->viewMatrix[7] = 0.0f; 
    renderer->viewMatrix[11] = 0.0f; 
    renderer->viewMatrix[15] = 1.0f;
    
    // Build projection matrix
    float f = 1.0f / tanf(camera.fov * 0.5f * M_PI / 180.0f);
    float nf = 1.0f / (camera.nearPlane - camera.farPlane);
    
    renderer->projectionMatrix[0] = f / camera.aspectRatio;
    renderer->projectionMatrix[4] = 0.0f;
    renderer->projectionMatrix[8] = 0.0f;
    renderer->projectionMatrix[12] = 0.0f;
    
    renderer->projectionMatrix[1] = 0.0f;
    renderer->projectionMatrix[5] = f;
    renderer->projectionMatrix[9] = 0.0f;
    renderer->projectionMatrix[13] = 0.0f;
    
    renderer->projectionMatrix[2] = 0.0f;
    renderer->projectionMatrix[6] = 0.0f;
    renderer->projectionMatrix[10] = (camera.farPlane + camera.nearPlane) * nf;
    renderer->projectionMatrix[14] = 2.0f * camera.farPlane * camera.nearPlane * nf;
    
    renderer->projectionMatrix[3] = 0.0f;
    renderer->projectionMatrix[7] = 0.0f;
    renderer->projectionMatrix[11] = -1.0f;
    renderer->projectionMatrix[15] = 0.0f;
}

SDL_Point renderer_world_to_screen(Renderer* renderer, Vec3 point) {
    // View transformation
    Vec4 viewSpace;
    viewSpace.x = renderer->viewMatrix[0] * point.x + renderer->viewMatrix[4] * point.y + 
                 renderer->viewMatrix[8] * point.z + renderer->viewMatrix[12];
    viewSpace.y = renderer->viewMatrix[1] * point.x + renderer->viewMatrix[5] * point.y + 
                 renderer->viewMatrix[9] * point.z + renderer->viewMatrix[13];
    viewSpace.z = renderer->viewMatrix[2] * point.x + renderer->viewMatrix[6] * point.y + 
                 renderer->viewMatrix[10] * point.z + renderer->viewMatrix[14];
    viewSpace.w = renderer->viewMatrix[3] * point.x + renderer->viewMatrix[7] * point.y + 
                 renderer->viewMatrix[11] * point.z + renderer->viewMatrix[15];
    
    // Projection transformation
    Vec4 clipSpace;
    clipSpace.x = renderer->projectionMatrix[0] * viewSpace.x + renderer->projectionMatrix[4] * viewSpace.y + 
                 renderer->projectionMatrix[8] * viewSpace.z + renderer->projectionMatrix[12] * viewSpace.w;
    clipSpace.y = renderer->projectionMatrix[1] * viewSpace.x + renderer->projectionMatrix[5] * viewSpace.y + 
                 renderer->projectionMatrix[9] * viewSpace.z + renderer->projectionMatrix[13] * viewSpace.w;
    clipSpace.z = renderer->projectionMatrix[2] * viewSpace.x + renderer->projectionMatrix[6] * viewSpace.y + 
                 renderer->projectionMatrix[10] * viewSpace.z + renderer->projectionMatrix[14] * viewSpace.w;
    clipSpace.w = renderer->projectionMatrix[3] * viewSpace.x + renderer->projectionMatrix[7] * viewSpace.y + 
                 renderer->projectionMatrix[11] * viewSpace.z + renderer->projectionMatrix[15] * viewSpace.w;
    
    // Perspective divide
    if (clipSpace.w != 0.0f) {
        clipSpace.x /= clipSpace.w;
        clipSpace.y /= clipSpace.w;
    }
    
    // Convert to screen coordinates
    SDL_Point screen;
    screen.x = (int)((clipSpace.x + 1.0f) * 0.5f * renderer->screenWidth);
    screen.y = (int)((1.0f - clipSpace.y) * 0.5f * renderer->screenHeight);
    return screen;
}

void renderer_draw_wireframe_line(Renderer* renderer, Vec3 start, Vec3 end) {
    SDL_Point p1 = renderer_world_to_screen(renderer, start);
    SDL_Point p2 = renderer_world_to_screen(renderer, end);
    SDL_RenderDrawLine(renderer->sdl_renderer, p1.x, p1.y, p2.x, p2.y);
}

void renderer_draw_wireframe_box(Renderer* renderer, Vec3 pos, Vec3 size, Quat rot) {
    // Define box vertices in local space
    Vec3 vertices[8];
    vertices[0] = vec3_create(-size.x/2, -size.y/2, -size.z/2);
    vertices[1] = vec3_create( size.x/2, -size.y/2, -size.z/2);
    vertices[2] = vec3_create( size.x/2,  size.y/2, -size.z/2);
    vertices[3] = vec3_create(-size.x/2,  size.y/2, -size.z/2);
    vertices[4] = vec3_create(-size.x/2, -size.y/2,  size.z/2);
    vertices[5] = vec3_create( size.x/2, -size.y/2,  size.z/2);
    vertices[6] = vec3_create( size.x/2,  size.y/2,  size.z/2);
    vertices[7] = vec3_create(-size.x/2,  size.y/2,  size.z/2);
    
    // Transform vertices to world space
    for (int i = 0; i < 8; i++) {
        vertices[i] = vec3_add(pos, vec3_quat_rotate(rot, vertices[i]));
    }
    
    // Draw edges
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 255, 0, 255);
    
    // Bottom face
    renderer_draw_wireframe_line(renderer, vertices[0], vertices[1]);
    renderer_draw_wireframe_line(renderer, vertices[1], vertices[2]);
    renderer_draw_wireframe_line(renderer, vertices[2], vertices[3]);
    renderer_draw_wireframe_line(renderer, vertices[3], vertices[0]);
    
    // Top face
    renderer_draw_wireframe_line(renderer, vertices[4], vertices[5]);
    renderer_draw_wireframe_line(renderer, vertices[5], vertices[6]);
    renderer_draw_wireframe_line(renderer, vertices[6], vertices[7]);
    renderer_draw_wireframe_line(renderer, vertices[7], vertices[4]);
    
    // Connecting edges
    renderer_draw_wireframe_line(renderer, vertices[0], vertices[4]);
    renderer_draw_wireframe_line(renderer, vertices[1], vertices[5]);
    renderer_draw_wireframe_line(renderer, vertices[2], vertices[6]);
    renderer_draw_wireframe_line(renderer, vertices[3], vertices[7]);
}

void renderer_draw_wireframe_sphere(Renderer* renderer, Vec3 pos, float radius) {
    const int segments = 16;
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 255, 0, 255);
    
    // Draw three circles in XY, YZ, and XZ planes
    for (int i = 0; i < segments; i++) {
        float theta1 = (float)i / segments * 2.0f * M_PI;
        float theta2 = (float)(i + 1) / segments * 2.0f * M_PI;
        
        // XY plane circle
        Vec3 p1 = vec3_add(pos, vec3_create(radius * cosf(theta1), radius * sinf(theta1), 0.0f));
        Vec3 p2 = vec3_add(pos, vec3_create(radius * cosf(theta2), radius * sinf(theta2), 0.0f));
        renderer_draw_wireframe_line(renderer, p1, p2);
        
        // YZ plane circle
        p1 = vec3_add(pos, vec3_create(0.0f, radius * cosf(theta1), radius * sinf(theta1)));
        p2 = vec3_add(pos, vec3_create(0.0f, radius * cosf(theta2), radius * sinf(theta2)));
        renderer_draw_wireframe_line(renderer, p1, p2);
        
        // XZ plane circle
        p1 = vec3_add(pos, vec3_create(radius * cosf(theta1), 0.0f, radius * sinf(theta1)));
        p2 = vec3_add(pos, vec3_create(radius * cosf(theta2), 0.0f, radius * sinf(theta2)));
        renderer_draw_wireframe_line(renderer, p1, p2);
    }
}

void renderer_draw_body(Renderer* renderer, RigidBody body) {
    switch (body.type) {
        case BODY_BOX:
            renderer_draw_wireframe_box(renderer, body.pos, body.size, body.rot);
            break;
        case BODY_SPHERE:
            renderer_draw_wireframe_sphere(renderer, body.pos, body.size.x);
            break;
    }
}

void renderer_draw_simulation(Renderer* renderer, const RigidBodySimulator* sim) {
    renderer_clear(renderer);
    
    // Draw coordinate axes
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 255, 0, 0, 255);  // X axis (red)
    renderer_draw_wireframe_line(renderer, 
        vec3_create(0, 0, 0), 
        vec3_create(1, 0, 0)
    );
    
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 255, 0, 255);  // Y axis (green)
    renderer_draw_wireframe_line(renderer,
        vec3_create(0, 0, 0),
        vec3_create(0, 1, 0)
    );
    
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 0, 255, 255);  // Z axis (blue)
    renderer_draw_wireframe_line(renderer,
        vec3_create(0, 0, 0),
        vec3_create(0, 0, 1)
    );
    
    // Draw all bodies
    for (int i = 0; i < sim->rigidBodyCount; i++) {
        renderer_draw_body(renderer, sim->rigidBodies[i]);
    }
    
    renderer_present(renderer);
}

void renderer_resize(Renderer* renderer, int width, int height) {
    renderer->screenWidth = width;
    renderer->screenHeight = height;
    renderer->camera.aspectRatio = (float)width / height;
    renderer_set_camera(renderer, renderer->camera);
}

} // namespace nik3dsim