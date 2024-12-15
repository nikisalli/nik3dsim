#include "nik3dsim.h"
#include <chrono>
using namespace nik3dsim;

int main() {
    nikModel m;
    nikData d;
    simulator_init(&m, (float[]){0, 0, -1}, 0.01f, 1, 0.1f);

    add_rigidbody(&m, BODY_BOX, (float[]){0.5f, 0.5f, 0.5f}, 1.0f, (float[]){1, 0, 0.5f}, (float[]){0, M_PI / 2, 0}, 0.001f, 1000.0f, 1, 1);

    add_rigidbody(&m, BODY_BOX, (float[]){0.5f, 0.5f, 0.5f}, 1.0f, (float[]){1, 3, 0.5f}, (float[]){0, M_PI / 2, 0}, 0.001f, 3.0f, 1, 1);

    add_staticbody(&m, BODY_PLANE, (float[]){0.0f, 0.0f, 0.0f}, (float[]){2, 0, 0}, (float[]){0, -M_PI * 0.02, 0}, 0.001f, 0.0f, 1, 1);

    data_init(&m, &d);

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 5e6; i++) {
        simulator_step(&m, &d);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    printf("%.0f steps/s\n", 5e12 / duration.count());

    return 0;
}