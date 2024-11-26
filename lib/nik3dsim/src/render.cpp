#include "render.hpp"
#include "math.hpp"
#include "types.hpp"
#include <cmath>
#include <cstddef>

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

void renderer_draw_wireframe_plane(Renderer* renderer, niknum pos[3], niknum rot[4]) {
    static const float SIZE = 10.0f;
    static const size_t NUM = 10;
    
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 255, 0, 255);
    
    // Create grid points in local space
    niknum local_point[3], world_point[3];
    niknum start[3], end[3];
    
    // Draw lines parallel to local X axis
    for (int i = 0; i <= NUM; i++) {
        float offset = -SIZE/2.0f + i * (SIZE/NUM);
        
        // Start point of line
        local_point[0] = -SIZE/2.0f;
        local_point[1] = offset;
        local_point[2] = 0.0f;
        
        // Transform to world space
        vec3_quat_rotate(world_point, rot, local_point);
        vec3_add(start, pos, world_point);
        
        // End point of line
        local_point[0] = SIZE/2.0f;
        local_point[1] = offset;
        local_point[2] = 0.0f;
        
        // Transform to world space
        vec3_quat_rotate(world_point, rot, local_point);
        vec3_add(end, pos, world_point);
        
        renderer_draw_wireframe_line(renderer, start, end);
    }
    
    // Draw lines parallel to local Y axis
    for (int i = 0; i <= NUM; i++) {
        float offset = -SIZE/2.0f + i * (SIZE/NUM);
        
        // Start point of line
        local_point[0] = offset;
        local_point[1] = -SIZE/2.0f;
        local_point[2] = 0.0f;
        
        // Transform to world space
        vec3_quat_rotate(world_point, rot, local_point);
        vec3_add(start, pos, world_point);
        
        // End point of line
        local_point[0] = offset;
        local_point[1] = SIZE/2.0f;
        local_point[2] = 0.0f;
        
        // Transform to world space
        vec3_quat_rotate(world_point, rot, local_point);
        vec3_add(end, pos, world_point);
        
        renderer_draw_wireframe_line(renderer, start, end);
    }
}

void renderer_draw_wireframe_capsule(Renderer* renderer, niknum pos[3], niknum size[3], niknum rot[4]) {
    const int segments = 16;  // Same as sphere for consistency
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 255, 0, 255);  // Green color like other shapes
    
    // Extract dimensions
    float radius = size[0];     // Radius
    float height = size[1];     // Total height
    float halfHeight = height * 0.5f;
    
    // Draw circles at different heights for the cylinder part
    niknum circle_points[segments + 1][3];
    niknum rotated_point[3];
    niknum world_point[3];
    
    // Generate points for one circle (in XY plane, will be rotated 90° around X to face Z)
    for (int i = 0; i <= segments; i++) {
        float angle = (float)i / segments * 2.0f * M_PI;
        circle_points[i][0] = radius * cosf(angle);
        circle_points[i][1] = radius * sinf(angle);
        circle_points[i][2] = 0.0f;
    }
    
    // Draw vertical lines of cylinder
    for (int i = 0; i < segments; i++) {
        // Bottom point
        niknum bottom[3] = {
            circle_points[i][0],
            circle_points[i][1],
            -halfHeight
        };
        
        // Top point
        niknum top[3] = {
            circle_points[i][0],
            circle_points[i][1],
            halfHeight
        };
        
        // Transform points to world space
        niknum world_bottom[3], world_top[3];
        vec3_quat_rotate(rotated_point, rot, bottom);
        vec3_add(world_bottom, pos, rotated_point);
        
        vec3_quat_rotate(rotated_point, rot, top);
        vec3_add(world_top, pos, rotated_point);
        
        // Draw vertical line
        renderer_draw_wireframe_line(renderer, world_bottom, world_top);
    }
    
    // Draw circles at top and bottom of cylinder
    for (int h = 0; h <= 1; h++) {  // h=0 for bottom, h=1 for top
        float z = (h == 0) ? -halfHeight : halfHeight;
        
        for (int i = 0; i < segments; i++) {
            niknum p1[3] = {
                circle_points[i][0],
                circle_points[i][1],
                z
            };
            
            niknum p2[3] = {
                circle_points[i + 1][0],
                circle_points[i + 1][1],
                z
            };
            
            // Transform points to world space
            niknum world_p1[3], world_p2[3];
            vec3_quat_rotate(rotated_point, rot, p1);
            vec3_add(world_p1, pos, rotated_point);
            
            vec3_quat_rotate(rotated_point, rot, p2);
            vec3_add(world_p2, pos, rotated_point);
            
            renderer_draw_wireframe_line(renderer, world_p1, world_p2);
        }
    }
    
    // Draw hemispheres at top and bottom
    for (int h = 0; h <= 1; h++) {  // h=0 for bottom, h=1 for top
        float baseZ = (h == 0) ? -halfHeight : halfHeight;
        float zSign = (h == 0) ? -1.0f : 1.0f;
        
        // Draw longitude lines
        for (int i = 0; i < segments; i++) {
            float phi = (float)i / segments * 2.0f * M_PI;
            float nextPhi = (float)(i + 1) / segments * 2.0f * M_PI;
            
            // Draw quarter circles at this longitude
            for (int j = 0; j < segments/4; j++) {
                float theta = (float)j / ((float)segments/4) * M_PI * 0.5f;
                float nextTheta = (float)(j + 1) / ((float)segments/4) * M_PI * 0.5f;
                
                niknum p1[3] = {
                    radius * cosf(phi) * cosf(theta),
                    radius * sinf(phi) * cosf(theta),
                    baseZ + zSign * radius * sinf(theta)
                };
                
                niknum p2[3] = {
                    radius * cosf(phi) * cosf(nextTheta),
                    radius * sinf(phi) * cosf(nextTheta),
                    baseZ + zSign * radius * sinf(nextTheta)
                };
                
                // Transform and draw longitude lines
                niknum world_p1[3], world_p2[3];
                vec3_quat_rotate(rotated_point, rot, p1);
                vec3_add(world_p1, pos, rotated_point);
                
                vec3_quat_rotate(rotated_point, rot, p2);
                vec3_add(world_p2, pos, rotated_point);
                
                renderer_draw_wireframe_line(renderer, world_p1, world_p2);
                
                // Draw latitude lines (connecting adjacent longitudes)
                niknum p3[3] = {
                    radius * cosf(nextPhi) * cosf(theta),
                    radius * sinf(nextPhi) * cosf(theta),
                    baseZ + zSign * radius * sinf(theta)
                };
                
                niknum world_p3[3];
                vec3_quat_rotate(rotated_point, rot, p3);
                vec3_add(world_p3, pos, rotated_point);
                
                renderer_draw_wireframe_line(renderer, world_p1, world_p3);
            }
        }
    }
}

void renderer_draw_wireframe_arrow(Renderer* renderer, niknum pos[3], niknum dir[3], float length, float head_length, float head_size) {
    // Normalize direction vector and scale to desired length
    niknum normalized_dir[3];
    vec3_normalize(normalized_dir, dir);
    vec3_scl(normalized_dir, normalized_dir, length);
    
    // Calculate arrow end point
    niknum end[3];
    vec3_add(end, pos, normalized_dir);
    
    // Draw main shaft
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 255, 255, 0, 255);  // Yellow color
    renderer_draw_wireframe_line(renderer, pos, end);
    
    // Calculate arrow head points
    // First, find two vectors perpendicular to direction vector
    niknum perpA[3], perpB[3];
    
    // Find first perpendicular vector
    if (fabs(normalized_dir[1]) < fabs(normalized_dir[0])) {
        perpA[0] = -normalized_dir[1];
        perpA[1] = normalized_dir[0];
        perpA[2] = 0;
    } else {
        perpA[0] = 0;
        perpA[1] = -normalized_dir[2];
        perpA[2] = normalized_dir[1];
    }
    vec3_normalize(perpA, perpA);
    vec3_scl(perpA, perpA, head_size);
    
    // Find second perpendicular vector using cross product
    vec3_cross(perpB, normalized_dir, perpA);
    vec3_normalize(perpB, perpB);
    vec3_scl(perpB, perpB, head_size);
    
    // Calculate arrow head base point
    niknum head_base[3];
    niknum head_dir[3];
    vec3_scl(head_dir, normalized_dir, -head_length);
    vec3_add(head_base, end, head_dir);
    
    // Calculate four points around base of arrow head
    niknum head_points[4][3];
    
    // Point 1
    vec3_add(head_points[0], head_base, perpA);
    vec3_add(head_points[0], head_points[0], perpB);
    
    // Point 2
    vec3_add(head_points[1], head_base, perpA);
    vec3_sub(head_points[1], head_points[1], perpB);
    
    // Point 3
    vec3_sub(head_points[2], head_base, perpA);
    vec3_sub(head_points[2], head_points[2], perpB);
    
    // Point 4
    vec3_sub(head_points[3], head_base, perpA);
    vec3_add(head_points[3], head_points[3], perpB);
    
    // Draw arrow head
    for (int i = 0; i < 4; i++) {
        // Draw lines from base points to tip
        renderer_draw_wireframe_line(renderer, head_points[i], end);
        
        // Draw base of arrow head
        renderer_draw_wireframe_line(renderer, head_points[i], head_points[(i + 1) % 4]);
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
        case BODY_PLANE:
            renderer_draw_wireframe_plane(renderer, data.pos, data.rot);
            break;
        case BODY_CAPSULE:
            renderer_draw_wireframe_capsule(renderer, data.pos, model.size, data.rot);
            break;
        default:
            break;
    }
}

void renderer_draw_static(Renderer* renderer, StaticBodyModel model) {
    switch (model.type) {
        case BODY_BOX:
            renderer_draw_wireframe_box(renderer, model.pos, model.size, model.rot);
            break;
        case BODY_SPHERE:
            renderer_draw_wireframe_sphere(renderer, model.pos, model.size[0]);
            break;
        case BODY_PLANE:
            renderer_draw_wireframe_plane(renderer, model.pos, model.rot);
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
    vec3_quat_rotate(world_point1, body1data->rot, constraint->r1);
    vec3_add(world_point1, world_point1, body1data->pos);
    
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
    vec3_quat_rotate(a1, body1data->rot, constraint->a1);
    vec3_normalize(a0, a0);
    vec3_normalize(a1, a1);
    vec3_addto(a0, body0data->pos);
    vec3_addto(a1, body1data->pos);
    
    // Draw green line between axis and body positions
    SDL_SetRenderDrawColor(renderer->sdl_renderer, 0, 0, 255, 255);  // Blue color
    renderer_draw_wireframe_line(renderer, body0data->pos, a0);
    renderer_draw_wireframe_line(renderer, body1data->pos, a1);
}

void renderer_draw_constraints(Renderer* renderer, const nikModel* model, const nikData* data) {
    // Draw all distance constraints
    for (size_t i = 0; i < model->positionalConstraintCount; i++) {
        renderer_draw_distance_constraint(renderer, &model->positionalConstraints[i], model->bodies, data->bodies);
    }
    
    // Draw all hinge constraints
    for (size_t i = 0; i < model->hingeConstraintCount; i++) {
        renderer_draw_hinge_constraint(renderer, &model->hingeConstraints[i], model->bodies, data->bodies);
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
        renderer_draw_body(renderer, model->bodies[i], data->bodies[i]);
    }

    for (int i = 0; i < model->staticBodyCount; i++) {
        renderer_draw_static(renderer, model->staticBodies[i]);
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