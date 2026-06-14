#include "rtutils.h"
#include "camera.h"
#include "scene.h"

#include <fstream>
#include <iostream>

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    std::ofstream out_file("output.ppm");
    if (!out_file.is_open()) {
        std::cerr << "Error: Could not open output file for writing.\n";
        return 1;
    }

    Scene world;

    world.add_horizontal_plane(0.0, MaterialType::Lambertian, color(0.5, 0.5, 0.5));

    point3 big_blue_center(0, 1.0, -5);
    point3 big_metal_center(2.5, 1.0, -5);
    point3 big_glass_center(-2.5, 1.0, -5);

    int grid_size = 100;
    double spacing = 1.5;
    double start_offset = -(grid_size * spacing) / 2.0;

    for (int a = 0; a < grid_size; ++a) {
        for (int b = 0; b < grid_size; ++b) {
            double choose_mat = random_double();
            
            double x_pos = start_offset + (a * spacing) + random_double(-0.4, 0.4);
            double z_pos = start_offset + (b * spacing) + random_double(-0.4, 0.4);
            double y_pos = 0.2; 

            point3 center(x_pos, y_pos, z_pos);

            double safety_radius = 1.3; 
            bool overlapping = ((center - big_blue_center).length() < safety_radius)  ||
                               ((center - big_metal_center).length() < safety_radius) ||
                               ((center - big_glass_center).length() < safety_radius);

            if (!overlapping) {
                if (choose_mat < 0.6) {
                    color albedo = random_vec3() * random_vec3();
                    world.add_sphere(center, 0.2, MaterialType::Lambertian, albedo);
                } 
                else if (choose_mat < 0.85) {
                    color albedo = random_vec3(0.5, 1);
                    double fuzz = random_double(0, 0.5);
                    world.add_sphere(center, 0.2, MaterialType::Metal, albedo, fuzz);
                } 
                else {
                    world.add_sphere(center, 0.2, MaterialType::Dielectric, color(1, 1, 1), 0.0, 1.5);
                }
            }
        }
    }

    world.add_sphere(big_blue_center, 1.0, MaterialType::Lambertian, color(0.1, 0.2, 0.5));
    world.add_sphere(big_metal_center, 1.0, MaterialType::Metal, color(0.8, 0.6, 0.2), 0.0);
    world.add_sphere(big_glass_center, 1.0, MaterialType::Dielectric, color(1.0, 1.0, 1.0), 0.0, 1.5);

    std::clog << "Building BVH acceleration tree...\n";
    world.build_bvh();
    std::clog << "BVH built successfully.\n";

    camera cam;

    cam.aspect_ratio      = 16.0 / 9.0;
    cam.image_width       = 400;
    cam.samples_per_pixel = 100;
    cam.max_depth         = 50;
    cam.vfov              = 20.0;

    cam.lookfrom = point3(-3, 3, 2);
    cam.lookat   = point3(0, 1.0, -5);  
    cam.vup      = vec3(0, 1, 0);

    cam.defocus_angle = 0.6; 
    cam.focus_dist    = 7.2;  

    cam.render(out_file, world);

    out_file.close();
    return 0;
}