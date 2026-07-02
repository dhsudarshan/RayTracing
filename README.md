# RayTracing

A CPU-based ray tracer written in modern C++.

This project was created as a learning exercise to understand the core concepts behind ray tracing and physically based rendering. Rather than using graphics APIs such as OpenGL or DirectX, every pixel is computed on the CPU through recursive ray tracing.

The renderer outputs images in the **PPM** image format and implements several fundamental rendering techniques, including multiple material models and BVH acceleration.

> **Status:** On hold. The project has achieved its primary learning goals and is not under active development. Future improvements may be added as time permits.

---

## Features

### Rendering

- Recursive ray tracing
- Anti-aliasing through stochastic sampling
- Gamma correction
- Configurable camera
- Depth of field
- Configurable recursion depth
- Multiple samples per pixel

### Scene Objects

- Spheres
- Triangles
- Infinite ground plane
- Axis-aligned boxes

### Materials

- Lambertian (Diffuse)
- Metal
- Dielectric (Glass)
- Diffuse light source

### Acceleration Structures

- Axis-Aligned Bounding Boxes (AABB)
- Bounding Volume Hierarchy (BVH)

---

## Example Scene

The default scene contains:

- A large ground plane
- Thousands of randomly generated spheres
- Metallic, diffuse, and dielectric materials
- An emissive light source
- A metallic box
- BVH acceleration for faster ray-object intersection

---

## Project Structure

```
RayTracing/
│
├── header/
│   ├── aabb.h
│   ├── camera.h
│   ├── color.h
│   ├── hittable.h
│   ├── interval.h
│   ├── material.h
│   ├── ray.h
│   ├── scene.h
│   ├── sphere.h
│   ├── triangle.h
│   ├── vec3.h
│   └── ...
│
├── src/
│   └── main.cpp
│
├── CMakeLists.txt
└── README.md
```

---

## Building

### Prerequisites

- C++20 compatible compiler
- CMake 3.20 or later
- Windows

### Using CMake (MSVC / Visual Studio)

```bash
git clone https://github.com/dhsudarshan/RayTracing.git
cd RayTracing

cmake -S . -B build
cmake --build build --config Release
```

### Using CMake with MinGW-w64 (g++)

```bash
git clone https://github.com/dhsudarshan/RayTracing.git
cd RayTracing

cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

> **Note:** If you previously configured the project using a different CMake generator, delete the existing `build` directory before configuring again.

---

## Running

```bash
./build/RayTracing
```

The rendered image is written to:

```
output.ppm
```

The generated image can be viewed with any application supporting the PPM image format or converted to PNG using external tools.

---

## Current Limitations

This renderer was built primarily for learning purposes and intentionally remains simple.

Current limitations include:

- CPU-only rendering
- No texture mapping
- No mesh importing (OBJ/STL)
- No multithreaded rendering
- No HDR environment maps
- No participating media or volumetrics
- PPM output only





