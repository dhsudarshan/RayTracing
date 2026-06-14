#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <algorithm>
#include "rtutils.h"
#include "ray.h"
#include "vec3.h"
#include "aabb.h"

enum class MaterialType { Lambertian, Metal, Dielectric };

struct SpherePrimitive {
    point3 center;
    double radius;
    MaterialType mat_type;
    color albedo;
    double fuzz;      
    double ir;        

    aabb bounding_box() const {
        return aabb(
            center - vec3(radius, radius, radius),
            center + vec3(radius, radius, radius)
        );
    }
};

struct PlanePrimitive {
    double y_height;
    MaterialType mat_type;
    color albedo;
    double fuzz;
    double ir;
};

struct SceneHitRecord {
    point3 p;
    vec3 normal;
    double t;
    bool front_face;
    MaterialType mat_type;
    color albedo;
    double fuzz;
    double ir;

    void set_face_normal(const ray& r, const vec3& outward_normal) {
        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

struct BVHNode {
    aabb box;
    int left_index = -1;  
    int right_index = -1; 
    int prim_index = -1;  
    bool is_leaf = false;
};

class Scene {
public:
    std::vector<SpherePrimitive> spheres;
    std::vector<BVHNode> bvh_nodes;
    std::vector<PlanePrimitive> planes; 
    int root_node_index = -1;

    void add_sphere(const point3& center, double radius, MaterialType type, const color& albedo, double fuzz = 0.0, double ir = 1.0) {
        spheres.push_back({center, radius, type, albedo, fuzz, ir});
    }

    void add_horizontal_plane(double y_height, MaterialType type, const color& albedo, double fuzz = 0.0, double ir = 1.0) {
        planes.push_back({y_height, type, albedo, fuzz, ir});
    }

    void build_bvh() {
        bvh_nodes.clear();
        if (spheres.empty()) return;

        std::vector<int> indices(spheres.size());
        for (size_t i = 0; i < spheres.size(); ++i) indices[i] = i;

        root_node_index = build_bvh_recursive(indices, 0, indices.size());
    }

    bool hit(const ray& r, double t_min, double t_max, SceneHitRecord& rec) const {
        bool hit_anything = false;
        auto closest_so_far = t_max;
        SceneHitRecord temp_rec;

        // 1. Analytical Infinite Plane Intersection Loop
        for (const auto& plane : planes) {
            auto dir_y = r.direction().y();
            if (std::abs(dir_y) < 1e-8) continue; 

            auto t = (plane.y_height - r.origin().y()) / dir_y;
            if (t < t_min || t > closest_so_far) continue;

            closest_so_far = t;
            hit_anything = true;

            temp_rec.t = t;
            temp_rec.p = r.at(t);
            vec3 outward_normal = vec3(0, 1, 0); 
            temp_rec.set_face_normal(r, outward_normal);
            temp_rec.mat_type = plane.mat_type;
            temp_rec.albedo = plane.albedo;
            temp_rec.fuzz = plane.fuzz;
            temp_rec.ir = plane.ir;
        }

        // 2. Stack-Based Flat BVH Acceleration Tree Traversal
        if (root_node_index != -1) {
            int node_stack[64]; 
            int stack_ptr = 0;
            node_stack[stack_ptr++] = root_node_index;

            while (stack_ptr > 0) {
                int curr_idx = node_stack[--stack_ptr];
                const auto& node = bvh_nodes[curr_idx];

                if (!node.box.hit(r, t_min, closest_so_far)) continue;

                if (node.is_leaf) {
                    if (hit_sphere_primitive(spheres[node.prim_index], r, t_min, closest_so_far, temp_rec)) {
                        hit_anything = true;
                        closest_so_far = temp_rec.t;
                        rec = temp_rec;
                    }
                } else {
                    node_stack[stack_ptr++] = node.right_index;
                    node_stack[stack_ptr++] = node.left_index;
                }
            }
        }

        if (hit_anything) rec = temp_rec;
        return hit_anything;
    }

private:
    int build_bvh_recursive(std::vector<int>& indices, size_t start, size_t end) {
        int current_node_idx = bvh_nodes.size();
        bvh_nodes.emplace_back(); 

        size_t span = end - start;
        aabb node_box;

        if (span == 1) {
            node_box = spheres[indices[start]].bounding_box();
            bvh_nodes[current_node_idx].box = node_box;
            bvh_nodes[current_node_idx].prim_index = indices[start];
            bvh_nodes[current_node_idx].is_leaf = true;
            return current_node_idx;
        }

        node_box = spheres[indices[start]].bounding_box();
        for (size_t i = start + 1; i < end; ++i) {
            node_box = surrounding_box(node_box, spheres[indices[i]].bounding_box());
        }

        vec3 size = node_box.max_pt - node_box.min_pt;
        int axis = 0;
        if (size.y() > size.x()) axis = 1;
        if (size.z() > size[axis]) axis = 2;

        auto comparator = [this, axis](int a, int b) {
            return spheres[a].bounding_box().min_pt[axis] < spheres[b].bounding_box().min_pt[axis];
        };

        std::sort(indices.begin() + start, indices.begin() + end, comparator);

        size_t mid = start + span / 2;
        
        int left_child = build_bvh_recursive(indices, start, mid);
        int right_child = build_bvh_recursive(indices, mid, end);

        bvh_nodes[current_node_idx].box = node_box;
        bvh_nodes[current_node_idx].left_index = left_child;
        bvh_nodes[current_node_idx].right_index = right_child;
        bvh_nodes[current_node_idx].is_leaf = false;

        return current_node_idx;
    }

    aabb surrounding_box(aabb box0, aabb box1) const {
        point3 small(std::min(box0.min_pt.x(), box1.min_pt.x()),
                     std::min(box0.min_pt.y(), box1.min_pt.y()),
                     std::min(box0.min_pt.z(), box1.min_pt.z()));

        point3 big(std::max(box0.max_pt.x(), box1.max_pt.x()),
                   std::max(box0.max_pt.y(), box1.max_pt.y()),
                   std::max(box0.max_pt.z(), box1.max_pt.z()));

        return aabb(small, big);
    }

    bool hit_sphere_primitive(const SpherePrimitive& sphere, const ray& r, double t_min, double t_max, SceneHitRecord& rec) const {
        vec3 oc = r.origin() - sphere.center;
        auto a = r.direction().length_squared();
        auto half_b = dot(oc, r.direction());
        auto c = oc.length_squared() - sphere.radius * sphere.radius;

        auto discriminant = half_b * half_b - a * c;
        if (discriminant < 0) return false;
        auto sqrtd = std::sqrt(discriminant);

        auto root = (-half_b - sqrtd) / a;
        if (root <= t_min || t_max <= root) {
            root = (-half_b + sqrtd) / a;
            if (root <= t_min || t_max <= root)
                return false;
        }

        rec.t = root;
        rec.p = r.at(root);
        vec3 outward_normal = (rec.p - sphere.center) / sphere.radius;
        rec.set_face_normal(r, outward_normal);
        rec.mat_type = sphere.mat_type;
        rec.albedo = sphere.albedo;
        rec.fuzz = sphere.fuzz;
        rec.ir = sphere.ir;

        return true;
    }
};

#endif