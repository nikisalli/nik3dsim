#include "nik3dsim.h"
#include <chrono>

using namespace nik3dsim;

int main() {
    nikModel m;
    nikData d;
    simulator_init(&m, (float[]){0, 0, -10}, 0.01f, 1, 0.0f);

    add_rigidbody(&m, BODY_BOX, (float[]){1.0f, 1.0f, 1.0f}, 0.0f, (float[]){0, 0, 0}, (float[]){0, 0, 0}, 0.001f, 0.0f, 1, 1);
    add_rigidbody(&m, BODY_BOX, (float[]){0.2f, 4.0f, 0.2f}, 1.0f, (float[]){0, 2, 0}, (float[]){0, 0, 0}, 0.001f, 0.0f, 1, 1);
    add_rigidbody(&m, BODY_BOX, (float[]){0.2f, 4.0f, 0.2f}, 1.0f, (float[]){0, 6, 0}, (float[]){0, 0, 0}, 0.001f, 0.0f, 1, 1);

    add_distance_constraint(&m, 0, 1, (float[]){0.0f, 0.0f, 0.0f}, (float[]){0.0f, -2.0f, 0.0f}, 0.0f, 0.0f);
    add_hinge_constraint(&m, 0, 1, (float[]){1.0f, 0.0f, 0.0f}, (float[]){1.0f, 0.0f, 0.0f}, 0.000000001f);

    add_distance_constraint(&m, 1, 2, (float[]){0.0f, 2.0f, 0.0f}, (float[]){0.0f, -2.0f, 0.0f}, 0.0f, 0.0f);
    add_hinge_constraint(&m, 1, 2, (float[]){1.0f, 0.0f, 0.0f}, (float[]){1.0f, 0.0f, 0.0f}, 0.000000001f);

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