#include "rtutils.h"
#include "camera.h"
#include "hittable_list.h"
#include "sphere.h"
#include "material.h"

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

    auto material_center = std::make_shared<lambertian>(color(0.3, 0.3, 0.3)); 
    auto material_ground = std::make_shared<lambertian>(color(0.8, 0.8, 0.0)); 

    hittable_list world;
    world.add(make_shared<sphere>(point3(0, 0, -5), 1.0, material_center));
    world.add(make_shared<sphere>(point3(0, -101, -5), 100.0, material_ground));

    camera cam;
    cam.aspect_ratio      = 16.0 / 9.0;
    cam.image_width       = 400;
    cam.samples_per_pixel = 100;
    cam.vfov = 130.0;

    cam.render(out_file, world);

    out_file.close();
    return 0;
}