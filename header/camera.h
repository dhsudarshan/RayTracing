#ifndef CAMERA_H
#define CAMERA_H

#include "rtutils.h"
#include "ray.h"
#include "vec3.h"
#include "scene.h"
#include <iostream>
#include <fstream>

class camera {
public:
    double aspect_ratio = 1.0;
    int image_width = 100;
    int samples_per_pixel = 10;
    int max_depth = 10;
    double vfov = 90;
    point3 lookfrom = point3(0,0,0);
    point3 lookat = point3(0,0,-1);
    vec3 vup = vec3(0,1,0);
    double defocus_angle = 0;
    double focus_dist = 10;

    void render(std::ofstream& out_file, const Scene& world) {
        initialize();

        out_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";
        #pragma omp parallel for schedule(dynamic)
        for (int j = 0; j < image_height; ++j) {
            std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
            for (int i = 0; i < image_width; ++i) {
                color pixel_color(0,0,0);
                for (int sample = 0; sample < samples_per_pixel; ++sample) {
                    ray r = get_ray(i, j);
                    pixel_color += ray_color(r, max_depth, world, true);
                }
                write_color(out_file, pixel_color, samples_per_pixel);
            }
        }
        std::clog << "\rDone.                 \n";
    }

private:
    int image_height;
    point3 center;
    point3 pixel00_loc;
    vec3 pixel_delta_u;
    vec3 pixel_delta_v;
    vec3 u, v, w;
    vec3 defocus_disk_u;
    vec3 defocus_disk_v;

    void initialize() {
        image_height = static_cast<int>(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        center = lookfrom;

        auto theta = degrees_to_radians(vfov);
        auto h = std::tan(theta/2);
        auto viewport_height = 2 * h * focus_dist;
        auto viewport_width = viewport_height * (static_cast<double>(image_width)/image_height);

        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        vec3 viewport_u = viewport_width * u;
        vec3 viewport_v = viewport_height * -v;

        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        auto viewport_upper_left = center - (focus_dist * w) - viewport_u/2 - viewport_v/2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j) const {
        auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
        auto pixel_sample = pixel_center + pixel_sample_square();

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
    }

    vec3 pixel_sample_square() const {
        auto px = -0.5 + random_double();
        auto py = -0.5 + random_double();
        return (px * pixel_delta_u) + (py * pixel_delta_v);
    }

    point3 defocus_disk_sample() const {
        auto p = random_in_unit_disk();
        return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    vec3 random_in_unit_disk() const {
        while (true) {
            auto p = vec3(random_double(-1,1), random_double(-1,1), 0);
            if (p.length_squared() < 1) return p;
        }
    }

    vec3 random_to_sphere(double radius, double distance_squared) const {
        auto r1 = random_double();
        auto r2 = random_double();
        auto z = 1.0 - r2 * (1.0 - std::sqrt(1.0 - (radius * radius / distance_squared)));
        auto phi = 2.0 * pi * r1;
        auto sin_theta = std::sqrt(1.0 - z * z);
        return vec3(sin_theta * std::cos(phi), sin_theta * std::sin(phi), z);
    }

    color emit_material(const SceneHitRecord& rec) const {
        if (rec.mat_type == MaterialType::DiffuseLight) {
            return rec.albedo;
        }
        return color(0, 0, 0);
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

    double reflectance(double cosine, double ref_idx) const {
        auto r0 = (1 - ref_idx) / (1 + ref_idx);
        r0 = r0 * r0;
        return r0 + (1 - r0) * std::pow((1 - cosine), 5);
    }

    vec3 random_unit_vector() const {
        while (true) {
            auto p = vec3(random_double(-1,1), random_double(-1,1), random_double(-1,1));
            auto lensq = p.length_squared();
            if (1e-160 < lensq && lensq <= 1.0)
                return p / std::sqrt(lensq);
        }
    }

    void write_color(std::ofstream& out, color pixel_color, int samples) {
        auto r = pixel_color.x();
        auto g = pixel_color.y();
        auto b = pixel_color.z();

        auto scale = 1.0 / samples;
        r *= scale;
        g *= scale;
        b *= scale;

        r = std::sqrt(r);
        g = std::sqrt(g);
        b = std::sqrt(b);

        out << static_cast<int>(256 * std::clamp(r, 0.0, 0.999)) << ' '
            << static_cast<int>(256 * std::clamp(g, 0.0, 0.999)) << ' '
            << static_cast<int>(256 * std::clamp(b, 0.0, 0.999)) << '\n';
    }

    color ray_color(const ray& r, int depth, const Scene& world, bool is_camera_ray) const {
        if (depth <= 0)
            return color(0,0,0);

        SceneHitRecord rec;

        if (!world.hit(r, 0.001, infinity, rec)) {
            return color(0.02, 0.02, 0.05); 
        }

        color color_from_emission = emit_material(rec);

        ray scattered;
        color attenuation;

        if (!scatter_material(rec, r, attenuation, scattered)) {
            return is_camera_ray ? color_from_emission : color(0,0,0);
        }

        color direct_light(0, 0, 0);

        if (!world.light_indices.empty() && rec.mat_type != MaterialType::Dielectric) {
            int light_idx = world.light_indices[static_cast<int>(random_double(0, world.light_indices.size() - 0.0001))];
            const auto& light_sphere = world.spheres[light_idx];

            vec3 to_light = light_sphere.center - rec.p;
            auto distance_squared = to_light.length_squared();
            vec3 direction_to_light = unit_vector(to_light);

            if (dot(direction_to_light, rec.normal) > 0) {
                vec3 w_axis = direction_to_light;
                vec3 a_axis = (std::abs(w_axis.x()) > 0.9) ? vec3(0,1,0) : vec3(1,0,0);
                vec3 v_axis = unit_vector(cross(w_axis, a_axis));
                vec3 u_axis = cross(w_axis, v_axis);

                vec3 implicit_dir = random_to_sphere(light_sphere.radius, distance_squared);
                vec3 shadow_ray_dir = implicit_dir.x() * u_axis + implicit_dir.y() * v_axis + implicit_dir.z() * w_axis;

                ray shadow_ray(rec.p, shadow_ray_dir);
                SceneHitRecord shadow_rec;

                if (world.hit(shadow_ray, 0.001, infinity, shadow_rec)) {
                    if (shadow_rec.mat_type == MaterialType::DiffuseLight) {
                        double cos_theta = dot(rec.normal, shadow_ray_dir);
                        
                        double cos_theta_max = std::sqrt(1.0 - (light_sphere.radius * light_sphere.radius / distance_squared));
                        double solid_angle_pdf = 1.0 / (2.0 * pi * (1.0 - cos_theta_max));

                        if (solid_angle_pdf > 0) {
                            direct_light = (attenuation * shadow_rec.albedo * cos_theta) / (solid_angle_pdf * pi);
                            direct_light = direct_light * static_cast<double>(world.light_indices.size());
                        }
                    }
                }
            }
        }

        bool is_specular = (rec.mat_type == MaterialType::Dielectric);
        color indirect_light = attenuation * ray_color(scattered, depth-1, world, is_specular);

        return color_from_emission + direct_light + indirect_light;
    }
};

#endif

