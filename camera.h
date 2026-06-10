#ifndef CAMERA_H
#define CAMERA_H

#include "rtutils.h"
#include "color.h"
#include "hittable.h"
#include "material.h"

#include <iostream>
#include <fstream>

class camera {
public:
    double aspect_ratio      = 16.0 / 9.0;
    int    image_width       = 400;
    int    samples_per_pixel = 100;
    int    max_depth         = 50;  
    double vfov = 90.0;

    void render(std::ofstream& out_file, const hittable& world) {
        initialize();

        out_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        for (int j = 0; j < image_height; ++j) {
            std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
            for (int i = 0; i < image_width; ++i) {
                color pixel_color(0,0,0);
                for (int sample = 0; sample < samples_per_pixel; ++sample) {
                    ray r = get_ray(i, j);
                    pixel_color += ray_color(r, max_depth, world);
                }
                color_utils::write_color(out_file, pixel_color, samples_per_pixel);
            }
        }
        std::clog << "\rDone. Image saved to output.ppm \n";
    }

private:
    int    image_height;
    point3 center;
    point3 pixel00_loc;
    vec3   pixel_delta_u;
    vec3   pixel_delta_v;

    void initialize() {
        image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        center = point3(0, 0, 0);

        auto focal_length = 1.0;
        auto theta = degrees_to_radians(vfov);
        auto h = std::tan(theta / 2.0);
        auto viewport_height = 2.0 * h * focal_length; 
        auto viewport_width = viewport_height * (double(image_width) / image_height);

        auto viewport_u = vec3(viewport_width, 0, 0);
        auto viewport_v = vec3(0, -viewport_height, 0);

        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        auto viewport_upper_left = center - vec3(0, 0, focal_length) - viewport_u/2 - viewport_v/2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);
    }

    ray get_ray(int i, int j) const {
        auto px = -0.5 + random_double();
        auto py = -0.5 + random_double();
        
        auto pixel_sample = pixel00_loc + ((i + px) * pixel_delta_u)  + ((j + py) * pixel_delta_v);

        auto ray_origin = center;
        auto ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
    }

    color ray_color(const ray& r, int depth, const hittable& world) const {
    if (depth <= 0)
        return color(0,0,0);

    hit_record rec;

    if (world.hit(r, 0.001, infinity, rec)) {
        ray scattered;
        color attenuation;
        if (rec.mat->scatter(r, rec, attenuation, scattered))
            return attenuation * ray_color(scattered, depth-1, world);
        return color(0,0,0);
    }

    vec3 unit_direction = unit_vector(r.direction());
    auto a = 0.5 * (unit_direction.y() + 1.0);
    return (1.0 - a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0);
}
};

#endif