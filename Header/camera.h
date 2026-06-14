#ifndef CAMERA_H
#define CAMERA_H

#include "rtutils.h"
#include "color.h"
#include "scene.h"
#include "ray.h"
#include "vec3.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

class camera {
public:
    double aspect_ratio      = 16.0 / 9.0;
    int    image_width       = 400;
    int    samples_per_pixel = 100;
    int    max_depth         = 50;  
    double vfov              = 90.0;

    point3 lookfrom  = point3(0,0,0);   
    point3 lookat    = point3(0,0,-1);  
    vec3   vup       = vec3(0,1,0);     

    double defocus_angle = 0.0;         
    double focus_dist    = 10.0;        

    void render(std::ofstream& out_file, const Scene& world) {
        initialize();

        out_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        std::vector<color> image_buffer(image_width * image_height);

        #pragma omp parallel for schedule(dynamic)
        for (int j = 0; j < image_height; ++j) {
            for (int i = 0; i < image_width; ++i) {
                color pixel_color(0,0,0);
                for (int sample = 0; sample < samples_per_pixel; ++sample) {
                    ray r = get_ray(i, j);
                    pixel_color += ray_color(r, max_depth, world);
                }
                image_buffer[j * image_width + i] = pixel_color;
            }
        }

        for (const auto& pixel_color : image_buffer) {
            color_utils::write_color(out_file, pixel_color, samples_per_pixel);
        }
        std::clog << "\rDone. Image saved to output.ppm \n";
    }

private:
    int    image_height;
    point3 center;
    point3 pixel00_loc;
    vec3   pixel_delta_u;
    vec3   pixel_delta_v;
    vec3   u, v, w;              
    vec3   defocus_disk_u;       
    vec3   defocus_disk_v;       

    void initialize() {
        image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        center = lookfrom;

        auto theta = degrees_to_radians(vfov);
        auto h = std::tan(theta / 2.0);
        auto viewport_height = 2.0 * h * focus_dist; 
        auto viewport_width = viewport_height * (double(image_width) / image_height);

        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        auto viewport_u = viewport_width * u;
        auto viewport_v = -viewport_height * v;

        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        auto viewport_upper_left = center - (focus_dist * w) - viewport_u/2 - viewport_v/2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2.0));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j) const {
        auto px = -0.5 + random_double();
        auto py = -0.5 + random_double();
        auto pixel_sample = pixel00_loc + ((i + px) * pixel_delta_u) + ((j + py) * pixel_delta_v);

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
    }

    point3 defocus_disk_sample() const {
        auto p = random_in_unit_disk();
        return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    static vec3 random_in_unit_disk() {
        while (true) {
            auto p = vec3(random_double(-1,1), random_double(-1,1), 0);
            if (p.length_squared() < 1)
                return p;
        }
    }

    bool scatter_material(const SceneHitRecord& rec, const ray& r_in, color& attenuation, ray& scattered) const {
        if (rec.mat_type == MaterialType::Lambertian) {
            auto scatter_direction = rec.normal + random_unit_vector();
            if (scatter_direction.near_zero())
                scatter_direction = rec.normal;
            scattered = ray(rec.p, scatter_direction);
            attenuation = rec.albedo;
            return true;
        } 
        else if (rec.mat_type == MaterialType::Metal) {
            vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);
            scattered = ray(rec.p, reflected + rec.fuzz * random_unit_vector());
            attenuation = rec.albedo;
            return (dot(scattered.direction(), rec.normal) > 0);
        } 
        else if (rec.mat_type == MaterialType::Dielectric) {
            attenuation = color(1.0, 1.0, 1.0);
            double refraction_ratio = rec.front_face ? (1.0 / rec.ir) : rec.ir;

            vec3 unit_direction = unit_vector(r_in.direction());
            double cos_theta = std::min(dot(-unit_direction, rec.normal), 1.0);
            double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);

            bool cannot_refract = refraction_ratio * sin_theta > 1.0;
            vec3 direction;

            if (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_double())
                direction = reflect(unit_direction, rec.normal);
            else
                direction = refract(unit_direction, rec.normal, refraction_ratio);

            scattered = ray(rec.p, direction);
            return true;
        }
        return false;
    }

    static double reflectance(double cosine, double ref_idx) {
        auto r0 = (1 - ref_idx) / (1 + ref_idx);
        r0 = r0 * r0;
        return r0 + (1 - r0) * std::pow((1 - cosine), 5);
    }

    color ray_color(const ray& r, int depth, const Scene& world) const {
        if (depth <= 0)
            return color(0,0,0);

        SceneHitRecord rec;

        if (world.hit(r, 0.001, infinity, rec)) {
            ray scattered;
            color attenuation;
            if (scatter_material(rec, r, attenuation, scattered))
                return attenuation * ray_color(scattered, depth-1, world);
            return color(0,0,0);
        }

        vec3 unit_direction = unit_vector(r.direction());
        auto a = 0.5 * (unit_direction.y() + 1.0);
        return (1.0 - a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0);
    }
};

#endif