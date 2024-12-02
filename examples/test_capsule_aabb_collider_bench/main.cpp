#include <chrono>
#include <iostream>
#include <random>
#include <vector>
#include "nik3dsim.h"

using namespace nik3dsim;
using namespace std::chrono;

struct TestCase {
    niknum capsulePos[3];
    niknum capsuleRot[4];
    niknum capsuleSize[3];
    niknum boxPos[3];
    niknum boxSize[3];
};

float randomFloat(float min, float max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);
    return dis(gen);
}

void generateRandomQuaternion(niknum quat[4]) {
    float yaw = randomFloat(-M_PI, M_PI);
    float pitch = randomFloat(-M_PI/2, M_PI/2);
    float roll = randomFloat(-M_PI, M_PI);
    
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    quat[0] = sr * cp * cy - cr * sp * sy;
    quat[1] = cr * sp * cy + sr * cp * sy;
    quat[2] = cr * cp * sy - sr * sp * cy;
    quat[3] = cr * cp * cy + sr * sp * sy;
    
    float mag = sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + 
                     quat[2] * quat[2] + quat[3] * quat[3]);
    for (int i = 0; i < 4; i++) {
        quat[i] /= mag;
    }
}

int main() {
    const int NUM_TEST_CASES = 1000;     // Number of pregenerated cases
    const int NUM_ITERATIONS = 10000000;  // Number of times to run through the test cases
    
    // Generate test cases
    std::vector<TestCase> testCases(NUM_TEST_CASES);
    for (auto& test : testCases) {
        for (int i = 0; i < 3; i++) {
            test.capsulePos[i] = randomFloat(-5.0f, 5.0f);
            test.boxPos[i] = randomFloat(-5.0f, 5.0f);
            test.boxSize[i] = randomFloat(0.5f, 2.0f);
        }
        
        test.capsuleSize[0] = randomFloat(0.1f, 1.0f);  // radius
        test.capsuleSize[1] = randomFloat(0.5f, 3.0f);  // height
        test.capsuleSize[2] = 0.0f;                     // unused
        
        generateRandomQuaternion(test.capsuleRot);
    }
    
    // Run the collision tests
    auto start = high_resolution_clock::now();
    
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        const TestCase& test = testCases[i % NUM_TEST_CASES];
        Contact contact = collide_capsule_aabb(
            test.capsulePos, test.capsuleRot, test.capsuleSize,
            test.boxPos, test.boxSize
        );
        (void)contact;
    }
    
    auto end = high_resolution_clock::now();
    duration<double> diff = end - start;
    
    double calls_per_second = NUM_ITERATIONS / diff.count();
    
    std::cout << "AABB Collision Test Configuration:\n";
    std::cout << "Unique test cases: " << NUM_TEST_CASES << "\n";
    std::cout << "Total iterations: " << NUM_ITERATIONS << "\n";
    std::cout << "Total Time: " << diff.count() << " seconds\n";
    std::cout << "Throughput: " << static_cast<int>(calls_per_second) << " calls/second\n";
    
    return 0;
}