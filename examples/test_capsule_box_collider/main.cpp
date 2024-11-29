#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <GL/glu.h>
#include <cmath>
#include "nik3dsim.h"

using namespace nik3dsim;

void drawBox(const float size[3]) {
    glPushMatrix();
    glScalef(size[0], size[1], size[2]);
    glBegin(GL_LINES);
    glVertex3f(-1, -1, -1); glVertex3f( 1, -1, -1);
    glVertex3f( 1, -1, -1); glVertex3f( 1,  1, -1);
    glVertex3f( 1,  1, -1); glVertex3f(-1,  1, -1);
    glVertex3f(-1,  1, -1); glVertex3f(-1, -1, -1);
    glVertex3f(-1, -1,  1); glVertex3f( 1, -1,  1);
    glVertex3f( 1, -1,  1); glVertex3f( 1,  1,  1);
    glVertex3f( 1,  1,  1); glVertex3f(-1,  1,  1);
    glVertex3f(-1,  1,  1); glVertex3f(-1, -1,  1);
    glVertex3f(-1, -1, -1); glVertex3f(-1, -1,  1);
    glVertex3f( 1, -1, -1); glVertex3f( 1, -1,  1);
    glVertex3f( 1,  1, -1); glVertex3f( 1,  1,  1);
    glVertex3f(-1,  1, -1); glVertex3f(-1,  1,  1);
    glEnd();
    glPopMatrix();
}

void drawSegment(const float start[3], const float end[3]) {
    glBegin(GL_LINES);
    glVertex3f(start[0], start[1], start[2]);
    glVertex3f(end[0], end[1], end[2]);
    glEnd();
}

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);

    SDL_Window* window = SDL_CreateWindow(
        "Box-Capsule Demo",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        2000, 1200,
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN
    );
    
    SDL_GLContext glContext = SDL_GL_CreateContext(window);
    
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    gluPerspective(45.0, 800.0/600.0, 0.1, 100.0);
    
    float cameraDistance = 10.0f;
    float cameraAngleX = 0.0f;
    float cameraAngleY = 0.0f;
    
    niknum boxSize[3] = {1.0f, 1.0f, 1.0f};
    niknum boxPos[3] = {0.0f, 0.0f, 0.0f};
    niknum boxRot[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    niknum boxInvRot[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    niknum capsuleSize[3] = {0.0f, 1.0f, 0.0f}; // radius, height, unused
    
    float time = 0.0f;
    bool running = true;
    
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            else if (event.type == SDL_MOUSEMOTION && (event.motion.state & SDL_BUTTON_LMASK)) {
                cameraAngleY += event.motion.xrel * -0.005f;
                cameraAngleX += event.motion.yrel * 0.005f;
            }
            else if (event.type == SDL_MOUSEWHEEL) {
                cameraDistance += event.wheel.y * -0.5f;
                if (cameraDistance < 2.0f) cameraDistance = 2.0f;
                if (cameraDistance > 20.0f) cameraDistance = 20.0f;
            }
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        
        gluLookAt(
            cameraDistance * sin(cameraAngleY) * cos(cameraAngleX),
            cameraDistance * sin(cameraAngleX),
            cameraDistance * cos(cameraAngleY) * cos(cameraAngleX),
            0, 0, 0,
            0, 1, 0
        );
        
        // Draw box
        glColor3f(1, 1, 1);
        drawBox(boxSize);
        
        // Update capsule position (figure-8 pattern)
        time += 0.016f;
        niknum capsulePos[3] = {
            2.0f * sinf(time * 0.7f),
            1.5f * cosf(time * 0.9f),
            2.0f * sinf(time * 1.1f)
        };
        niknum capsuleRot[4] = {0.0f, 0.0f, 0.0f, 1.0f}; // No rotation for simplicity
        
        // Draw capsule segment (just visualizing the central axis)
        niknum _capsuleDir[3] = {0, 0, 1};
        vec3_quat_rotate(_capsuleDir, capsuleRot, _capsuleDir);
        niknum _segStart[3], _segEnd[3];
        vec3_addscl(_segStart, capsulePos, _capsuleDir, capsuleSize[1]/2);
        vec3_addscl(_segEnd, capsulePos, _capsuleDir, -capsuleSize[1]/2);
        
        glColor3f(0, 1, 0);
        drawSegment(_segStart, _segEnd);
        
        // Get collision information
        Contact contact = collide_capsule_box(
            capsulePos, capsuleRot, capsuleSize,
            boxPos, boxRot, boxInvRot, boxSize
        );
        
        // Draw collision normal and contact point
        if (contact.depth > -capsuleSize[0]) {  // If collision detected
            glColor3f(1, 0, 0);
            glPointSize(5.0f);
            glBegin(GL_POINTS);
            glVertex3f(contact.pos[0], contact.pos[1], contact.pos[2]);
            glEnd();
            
            // Draw normal
            glBegin(GL_LINES);
            glVertex3f(contact.pos[0], contact.pos[1], contact.pos[2]);
            glVertex3f(
                contact.pos[0] + contact.n[0] * contact.depth,
                contact.pos[1] + contact.n[1] * contact.depth,
                contact.pos[2] + contact.n[2] * contact.depth
            );
            glEnd();
        }
        
        SDL_GL_SwapWindow(window);
        SDL_Delay(16);
    }
    
    SDL_GL_DeleteContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();
    
    return 0;
}