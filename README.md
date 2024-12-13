# nik3dsim: Position-Based 3D Physics Engine

A lightweight physics engine for real-time 3D rigidbody simulation made for efficient massively parallel reinforcement learning with:
- NO third party dependencies apart from opengl and SDL for the renderer
- very fast custom written primitive colliders
- branchless and single thread design made for massively parallel simulations

<img src="https://github.com/user-attachments/assets/261d4e85-bc24-4642-bfc1-c0cf5482c05e" height="200">
<img src="https://github.com/user-attachments/assets/7d2e197a-33f1-467f-a981-4050d9c62d2a" height="200">
<img src="https://github.com/user-attachments/assets/3b3fd9ce-b24f-4e8d-a096-2daffacbfecc" height="200">
<img src="https://github.com/user-attachments/assets/064b1540-ed6f-4d32-9a85-d6aa93fbd362" height="200">
<img src="https://github.com/user-attachments/assets/62311d03-0e33-41a6-8fd2-fb03605533ac" height="200">


## Overview

nik3dsim uses Position-Based Dynamics, treating physics constraints as positional corrections rather than force-based interactions. This approach provides unconditional stability and very fast and robust simulations.

## Features

- Rigid body dynamics with accurate inertia tensors
- Multiple collision primitives:
  - Spheres
  - Boxes (OBB)
  - Axis-aligned boxes (AABB)
  - Capsules
  - Static planes
- Constraints:
  - Distance constraints
  - Hinge joints
  - Contact constraints with friction (static and dynamic)
- Stable collision resolution using PBD
- OpenGL visualization with SDL2

## Building

```bash
mkdir build && cd build
cmake ..
make
../bin/test_softbody
```

Requires:
- C++11 compiler
- SDL2
- OpenGL

## Usage

Check the example folder for many examples for benchmarking and showcasing.

## Acknowledgments
This implementation is based on the Position Based Dynamics approach described in "Position Based Dynamics" by Matthias Müller et al. Thanks to their pioneering work in making physics simulations more stable and efficient.
