#include "render.hpp"
#include "math.hpp"
#include "types.hpp"
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
    niknum position[3] = {5.0f, 5.0f, 5.0f};
    niknum target[3] = {0.0f, 0.0f, 0.0f};
    niknum up[3] = {0.0f, 1.0f, 0.0f};
    
    for(int i = 0; i < 3; i++) {
        renderer->camera.position[i] = position[i];
        renderer->camera.target[i] = target[i];
        renderer->camera.up[i] = up[i];
    }
    
    renderer->camera.fov = 60.0f;
    renderer->camera.aspectRatio = (float)width / height;
    renderer->camera.nearPlane = 0.1f;
    renderer->camera.farPlane = 100.0f;
    
    renderer_set_camera(renderer, renderer->camera);
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
    niknum z[3], x[3], y[3], neg_pos[3], temp[3];
    
    vec3_sub(temp, camera.position, camera.target);
    vec3_normalize(z, temp);
    
    vec3_cross(x, camera.up, z);
    vec3_normalize(x, x);
    
    vec3_cross(y, z, x);
    
    vec3_scl(neg_pos, camera.position, -1.0f);
    
    // Build view matrix (row-major)
    renderer->viewMatrix[0] = x[0];  
    renderer->viewMatrix[4] = x[1];  
    renderer->viewMatrix[8] = x[2];  
    renderer->viewMatrix[12] = vec3_dot(x, neg_pos);
    
    renderer->viewMatrix[1] = y[0];  
    renderer->viewMatrix[5] = y[1];  
    renderer->viewMatrix[9] = y[2];  
    renderer->viewMatrix[13] = vec3_dot(y, neg_pos);
    
    renderer->viewMatrix[2] = z[0];  
    renderer->viewMatrix[6] = z[1];  
    renderer->viewMatrix[10] = z[2]; 
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

SDL_Point renderer_world_to_screen(Renderer* renderer, const niknum point[3]) {
    // View transformation
    niknum viewSpace[4];
    viewSpace[0] = renderer->viewMatrix[0] * point[0] + renderer->viewMatrix[4] * point[1] + 
                   renderer->viewMatrix[8] * point[2] + renderer->viewMatrix[12];
    viewSpace[1] = renderer->viewMatrix[1] * point[0] + renderer->viewMatrix[5] * point[1] + 
                   renderer->viewMatrix[9] * point[2] + renderer->viewMatrix[13];
    viewSpace[2] = renderer->viewMatrix[2] * point[0] + renderer->viewMatrix[6] * point[1] + 
                   renderer->viewMatrix[10] * point[2] + renderer->viewMatrix[14];
    viewSpace[3] = renderer->viewMatrix[3] * point[0] + renderer->viewMatrix[7] * point[1] + 
                   renderer->viewMatrix[11] * point[2] + renderer->viewMatrix[15];
    
    // Projection transformation
    niknum clipSpace[4];
    clipSpace[0] = renderer->projectionMatrix[0] * viewSpace[0] + renderer->projectionMatrix[4] * viewSpace[1] + 
                   renderer->projectionMatrix[8] * viewSpace[2] + renderer->projectionMatrix[12] * viewSpace[3];
    clipSpace[1] = renderer->projectionMatrix[1] * viewSpace[0] + renderer->projectionMatrix[5] * viewSpace[1] + 
                   renderer->projectionMatrix[9] * viewSpace[2] + renderer->projectionMatrix[13] * viewSpace[3];
    clipSpace[2] = renderer->projectionMatrix[2] * viewSpace[0] + renderer->projectionMatrix[6] * viewSpace[1] + 
                   renderer->projectionMatrix[10] * viewSpace[2] + renderer->projectionMatrix[14] * viewSpace[3];
    clipSpace[3] = renderer->projectionMatrix[3] * viewSpace[0] + renderer->projectionMatrix[7] * viewSpace[1] + 
                   renderer->projectionMatrix[11] * viewSpace[2] + renderer->projectionMatrix[15] * viewSpace[3];
    
    // Perspective divide
    if (clipSpace[3] != 0.0f) {
        clipSpace[0] /= clipSpace[3];
        clipSpace[1] /= clipSpace[3];
    }
    
    // Convert to screen coordinates
    SDL_Point screen;
    screen.x = (int)((clipSpace[0] + 1.0f) * 0.5f * renderer->screenWidth);
    screen.y = (int)((1.0f - clipSpace[1]) * 0.5f * renderer->screenHeight);
    return screen;
}

void renderer_draw_wireframe_line(Renderer* renderer, const niknum start[3], const niknum end[3]) {
    SDL_Point p1 = renderer_world_to_screen(renderer, start);
    SDL_Point p2 = renderer_world_to_screen(renderer, end);
    SDL_RenderDrawLine(renderer->sdl_renderer, p1.x, p1.y, p2.x, p2.y);
}

void renderer_draw_wireframe_box(Renderer* renderer, niknum pos[3], niknum size[3], niknum rot[4]) {
    // Define box vertices in local space
    niknum vertices[8][3];
    vertices[0][0] = -size[0]/2; vertices[0][1] = -size[1]/2; vertices[0][2] = -size[2]/2;
    vertices[1][0] =  size[0]/2; vertices[1][1] = -size[1]/2; vertices[1][2] = -size[2]/2;
    vertices[2][0] =  size[0]/2; vertices[2][1] =  size[1]/2; vertices[2][2] = -size[2]/2;
    vertices[3][0] = -size[0]/2; vertices[3][1] =  size[1]/2; vertices[3][2] = -size[2]/2;
    vertices[4][0] = -size[0]/2; vertices[4][1] = -size[1]/2; vertices[4][2] =  size[2]/2;
    vertices[5][0] =  size[0]/2; vertices[5][1] = -size[1]/2; vertices[5][2] =  size[2]/2;
    vertices[6][0] =  size[0]/2; vertices[6][1] =  size[1]/2; vertices[6][2] =  size[2]/2;
    vertices[7][0] = -size[0]/2; vertices[7][1] =  size[1]/2; vertices[7][2] =  size[2]/2;
    
    // Transform vertices to world space
    niknum rotated[3];
    for (int i = 0; i < 8; i++) {
        vec3_quat_rotate(rotated, rot, vertices[i]);
        vec3_add(vertices[i], pos, rotated);
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

void renderer_draw_wireframe_sphere(Renderer* renderer, niknum pos[3], float radius) {
    const int segments = 16;
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 255, 0, 255);
    
    // Draw three circles in XY, YZ, and XZ planes
    niknum p1[3], p2[3];
    for (int i = 0; i < segments; i++) {
        float theta1 = (float)i / segments * 2.0f * M_PI;
        float theta2 = (float)(i + 1) / segments * 2.0f * M_PI;
        
        // XY plane circle
        p1[0] = pos[0] + radius * cosf(theta1);
        p1[1] = pos[1] + radius * sinf(theta1);
        p1[2] = pos[2];
        
        p2[0] = pos[0] + radius * cosf(theta2);
        p2[1] = pos[1] + radius * sinf(theta2);
        p2[2] = pos[2];
        renderer_draw_wireframe_line(renderer, p1, p2);
        
        // YZ plane circle
        p1[0] = pos[0];
        p1[1] = pos[1] + radius * cosf(theta1);
        p1[2] = pos[2] + radius * sinf(theta1);
        
        p2[0] = pos[0];
        p2[1] = pos[1] + radius * cosf(theta2);
        p2[2] = pos[2] + radius * sinf(theta2);
        renderer_draw_wireframe_line(renderer, p1, p2);
        
        // XZ plane circle
        p1[0] = pos[0] + radius * cosf(theta1);
        p1[1] = pos[1];
        p1[2] = pos[2] + radius * sinf(theta1);
        
        p2[0] = pos[0] + radius * cosf(theta2);
        p2[1] = pos[1];
        p2[2] = pos[2] + radius * sinf(theta2);
        renderer_draw_wireframe_line(renderer, p1, p2);
    }
}

void handle_mouse_events(SDL_Event& event, Camera& camera, MouseState& mouseState) {
    static const float ROTATION_SPEED = 0.3f;    // Degrees per pixel
    static const float ZOOM_SPEED = 1.0f;
    
    switch (event.type) {
        case SDL_MOUSEBUTTONDOWN: {
            if (event.button.button == SDL_BUTTON_LEFT) {
                mouseState.leftButtonDown = true;
            }
            else if (event.button.button == SDL_BUTTON_RIGHT) {
                mouseState.rightButtonDown = true;
            }
            mouseState.lastX = event.button.x;
            mouseState.lastY = event.button.y;
            break;
        }
        
        case SDL_MOUSEBUTTONUP: {
            if (event.button.button == SDL_BUTTON_LEFT) {
                mouseState.leftButtonDown = false;
            }
            else if (event.button.button == SDL_BUTTON_RIGHT) {
                mouseState.rightButtonDown = false;
            }
            break;
        }
        
        case SDL_MOUSEWHEEL: {
            mouseState.dist -= event.wheel.y * ZOOM_SPEED;
            mouseState.dist = fmax(0.1f, mouseState.dist); // Prevent negative or zero distance
            
            // Update camera position immediately after distance change
            double camxoffset = mouseState.dist * sin(mouseState.azim * M_PI / 180.0) * cos(mouseState.elev * M_PI / 180.0);
            double camyoffset = mouseState.dist * cos(mouseState.azim * M_PI / 180.0) * cos(mouseState.elev * M_PI / 180.0);
            double camzoffset = mouseState.dist * sin(mouseState.elev * M_PI / 180.0);
            
            camera.position[0] = camera.target[0] + camxoffset;
            camera.position[1] = camera.target[1] + camyoffset;
            camera.position[2] = camera.target[2] + camzoffset;
            break;
        }
        
        case SDL_MOUSEMOTION: {
            int deltaX = event.motion.x - mouseState.lastX;
            int deltaY = event.motion.y - mouseState.lastY;
            
            if (mouseState.leftButtonDown) {
                // Update azimuth and elevation angles
                mouseState.azim += deltaX * ROTATION_SPEED;
                mouseState.elev += deltaY * ROTATION_SPEED;
                
                // Clamp elevation to prevent gimbal lock
                mouseState.elev = fmax(-89.0f, fmin(89.0f, mouseState.elev));
                
                // Keep azimuth in [0, 360) range
                while (mouseState.azim >= 360.0f) mouseState.azim -= 360.0f;
                while (mouseState.azim < 0.0f) mouseState.azim += 360.0f;
            }
            
            // Update camera position based on spherical coordinates
            double camxoffset = mouseState.dist * sin(mouseState.azim * M_PI / 180.0) * cos(mouseState.elev * M_PI / 180.0);
            double camyoffset = mouseState.dist * cos(mouseState.azim * M_PI / 180.0) * cos(mouseState.elev * M_PI / 180.0);
            double camzoffset = mouseState.dist * sin(mouseState.elev * M_PI / 180.0);
            
            camera.position[0] = camera.target[0] + camxoffset;
            camera.position[1] = camera.target[1] + camyoffset;
            camera.position[2] = camera.target[2] + camzoffset;
            
            mouseState.lastX = event.motion.x;
            mouseState.lastY = event.motion.y;
            break;
        }
    }
}

void renderer_draw_body(Renderer* renderer, RigidBodyModel model, RigidBodyData data) {
    switch (model.type) {
        case BODY_BOX:
            renderer_draw_wireframe_box(renderer, data.pos, model.size, data.rot);
            break;
        case BODY_SPHERE:
            renderer_draw_wireframe_sphere(renderer, data.pos, model.size[0]);
            break;
        default:
            break;
    }
}

void renderer_draw_distance_constraint(Renderer* renderer, const DistanceConstraint* constraint, const RigidBodyModel* models, const RigidBodyData* data) {
    // Get the two connected bodies
    const RigidBodyModel* body0model = &models[constraint->b0];
    const RigidBodyModel* body1model = &models[constraint->b1];
    const RigidBodyData* body0data = &data[constraint->b0];
    const RigidBodyData* body1data = &data[constraint->b1];
    
    // Transform attachment points from local to world space
    niknum world_point0[3], world_point1[3];
    
    // Transform r0 by body0's rotation and position
    vec3_quat_rotate(world_point0, body0data->rot, constraint->r0);
    vec3_add(world_point0, world_point0, body0data->pos);
    
    // Transform r1 by body1's rotation and position
    vec3_quat_rotate(world_point1, body0data->rot, constraint->r1);
    vec3_add(world_point1, world_point1, body0data->pos);
    
    // Draw red line between attachment points
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 255, 0, 0, 255);  // Red color
    renderer_draw_wireframe_line(renderer, world_point0, world_point1);
}

void renderer_draw_hinge_constraint(Renderer* renderer, const HingeConstraint* constraint, const RigidBodyModel* models, const RigidBodyData* data) {
    // Get the two connected bodies
    const RigidBodyModel* body0model = &models[constraint->b0];
    const RigidBodyModel* body1model = &models[constraint->b1];
    const RigidBodyData* body0data = &data[constraint->b0];
    const RigidBodyData* body1data = &data[constraint->b1];
    
    // Transform axis by body rotations
    niknum a0[3], a1[3];
    vec3_quat_rotate(a0, body0data->rot, constraint->a0);
    vec3_quat_rotate(a1, body0data->rot, constraint->a1);
    vec3_normalize(a0, a0);
    vec3_normalize(a1, a1);
    vec3_addto(a0, body0data->pos);
    vec3_addto(a1, body0data->pos);
    
    // Draw green line between axis and body positions
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 0, 255, 255);  // Blue color
    renderer_draw_wireframe_line(renderer, body0data->pos, a0);
    renderer_draw_wireframe_line(renderer, body0data->pos, a1);
}

void renderer_draw_constraints(Renderer* renderer, const nikModel* model, const nikData* data) {
    // Draw all distance constraints
    for (size_t i = 0; i < model->positionalConstraintCount; i++) {
        renderer_draw_distance_constraint(renderer, &model->positionalConstraints[i], model->rigidBodies, data->rigidBodies);
    }
    
    // Draw all hinge constraints
    for (size_t i = 0; i < model->hingeConstraintCount; i++) {
        renderer_draw_hinge_constraint(renderer, &model->hingeConstraints[i], model->rigidBodies, data->rigidBodies);
    }
}

// Modify renderer_draw_simulation to include constraint drawing:
void renderer_draw_simulation(Renderer* renderer, const nikModel* model, const nikData* data) {
    // Draw coordinate axes
    niknum origin[3] = {0, 0, 0};
    niknum x_axis[3] = {1, 0, 0};
    niknum y_axis[3] = {0, 1, 0};
    niknum z_axis[3] = {0, 0, 1};
    
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 255, 0, 0, 255);  // X axis (red)
    renderer_draw_wireframe_line(renderer, origin, x_axis);
    
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 255, 0, 255);  // Y axis (green)
    renderer_draw_wireframe_line(renderer, origin, y_axis);
    
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 0, 255, 255);  // Z axis (blue)
    renderer_draw_wireframe_line(renderer, origin, z_axis);
    
    // Draw all bodies
    for (int i = 0; i < model->rigidBodyCount; i++) {
        renderer_draw_body(renderer, model->rigidBodies[i], data->rigidBodies[i]);
    }
    
    // Draw all constraints
    renderer_draw_constraints(renderer, model, data);
    
    renderer_present(renderer);

    renderer_clear(renderer);
}

void renderer_resize(Renderer* renderer, int width, int height) {
    renderer->screenWidth = width;
    renderer->screenHeight = height;
    renderer->camera.aspectRatio = (float)width / height;
    renderer_set_camera(renderer, renderer->camera);
}

} // namespace nik3dsim