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

void drawSphere(float radius, int subdivisions) {
    for(int i = 0; i < subdivisions; ++i) {
        float lat0 = M_PI * (-0.5 + (float) (i) / subdivisions);
        float lat1 = M_PI * (-0.5 + (float) (i + 1) / subdivisions);
        
        glBegin(GL_LINE_STRIP);
        for(int j = 0; j <= subdivisions; ++j) {
            float lng = 2 * M_PI * (float) (j) / subdivisions;
            float x = cos(lng) * cos(lat0);
            float y = sin(lat0);
            float z = sin(lng) * cos(lat0);
            glVertex3f(x * radius, y * radius, z * radius);
            
            x = cos(lng) * cos(lat1);
            y = sin(lat1);
            z = sin(lng) * cos(lat1);
            glVertex3f(x * radius, y * radius, z * radius);
        }
        glEnd();
    }
}

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);

    SDL_Window* window = SDL_CreateWindow(
        "Sphere-Box Collision Demo",
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
    niknum sphereSize[3] = {0.5f, 0.0f, 0.0f}; // radius in first component
    niknum spherePos[3] = {2.0f, 0.0f, 0.0f}; // starting position
    
    const float MOVE_SPEED = 0.1f; // Speed of sphere movement
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

        // Handle keyboard input for sphere movement
        const Uint8* keystate = SDL_GetKeyboardState(NULL);
        if (keystate[SDL_SCANCODE_W]) spherePos[2] -= MOVE_SPEED;
        if (keystate[SDL_SCANCODE_S]) spherePos[2] += MOVE_SPEED;
        if (keystate[SDL_SCANCODE_A]) spherePos[0] -= MOVE_SPEED;
        if (keystate[SDL_SCANCODE_D]) spherePos[0] += MOVE_SPEED;
        if (keystate[SDL_SCANCODE_UP]) spherePos[1] += MOVE_SPEED;
        if (keystate[SDL_SCANCODE_DOWN]) spherePos[1] -= MOVE_SPEED;

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
        
        // Draw sphere
        glPushMatrix();
        glTranslatef(spherePos[0], spherePos[1], spherePos[2]);
        glColor3f(0, 1, 0);
        drawSphere(sphereSize[0], 12);
        glPopMatrix();
        
        // Get collision information
        Contact contacts[1];
        int numcon = collide_sphere_box(contacts,
            spherePos, sphereSize,
            boxPos, boxRot, boxSize
        );
        
        // Draw collision normal and contact points
        glColor3f(1, 0, 0);
        glPointSize(5.0f);
        glBegin(GL_POINTS);
        glVertex3f(contacts[0].pos0[0], contacts[0].pos0[1], contacts[0].pos0[2]);
        glEnd();
        
        // Draw normal
        glBegin(GL_LINES);
        glVertex3f(contacts[0].pos0[0], contacts[0].pos0[1], contacts[0].pos0[2]);
        glVertex3f(
            contacts[0].pos1[0],
            contacts[0].pos1[1],
            contacts[0].pos1[2]
        );
        glEnd();
        
        SDL_GL_SwapWindow(window);
        SDL_Delay(16);
    }
    
    SDL_GL_DeleteContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();
    
    return 0;
}