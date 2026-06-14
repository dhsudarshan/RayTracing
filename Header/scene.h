#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <algorithm>
#include <cmath>
#include "rtutils.h"
#include "ray.h"
#include "vec3.h"
#include "aabb.h"

enum class MaterialType { Lambertian, Metal, Dielectric, DiffuseLight };

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

struct TrianglePrimitive {
    point3 v0, v1, v2;
    vec3 normal;
    MaterialType mat_type;
    color albedo;
    double fuzz;
    double ir;

    aabb bounding_box() const {
        point3 min_pt(
            std::min({v0.x(), v1.x(), v2.x()}) - 0.0001,
            std::min({v0.y(), v1.y(), v2.y()}) - 0.0001,
            std::min({v0.z(), v1.z(), v2.z()}) - 0.0001
        );
        point3 max_pt(
            std::max({v0.x(), v1.x(), v2.x()}) + 0.0001,
            std::max({v0.y(), v1.y(), v2.y()}) + 0.0001,
            std::max({v0.z(), v1.z(), v2.z()}) + 0.0001
        );
        return aabb(min_pt, max_pt);
    }
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
    bool is_sphere = true;
    bool is_leaf = false;
};

class Scene {
public:
    std::vector<SpherePrimitive> spheres;
    std::vector<TrianglePrimitive> triangles;
    std::vector<BVHNode> bvh_nodes;
    std::vector<PlanePrimitive> planes; 
    std::vector<int> light_indices; 
    int root_node_index = -1;

    void add_sphere(const point3& center, double radius, MaterialType type, const color& albedo, double fuzz = 0.0, double ir = 1.0) {
        spheres.push_back({center, radius, type, albedo, fuzz, ir});
        if (type == MaterialType::DiffuseLight) {
            light_indices.push_back(spheres.size() - 1);
        }
    }

    void add_horizontal_plane(double y_height, MaterialType type, const color& albedo, double fuzz = 0.0, double ir = 1.0) {
        planes.push_back({y_height, type, albedo, fuzz, ir});
    }

    void add_triangle(const point3& v0, const point3& v1, const point3& v2, MaterialType type, const color& albedo, double fuzz = 0.0, double ir = 1.0) {
        vec3 normal = unit_vector(cross(v1 - v0, v2 - v0));
        triangles.push_back({v0, v1, v2, normal, type, albedo, fuzz, ir});
    }

    void add_box(const point3& p0, const point3& p1, MaterialType type, const color& albedo, double fuzz = 0.0, double ir = 1.0) {
        double min_x = std::min(p0.x(), p1.x());
        double max_x = std::max(p0.x(), p1.x());
        double min_y = std::min(p0.y(), p1.y());
        double max_y = std::max(p0.y(), p1.y());
        double min_z = std::min(p0.z(), p1.z());
        double max_z = std::max(p0.z(), p1.z());

        point3 v000(min_x, min_y, min_z);
        point3 v100(max_x, min_y, min_z);
        point3 v110(max_x, max_y, min_z);
        point3 v010(min_x, max_y, min_z);
        point3 v001(min_x, min_y, max_z);
        point3 v101(max_x, min_y, max_z);
        point3 v111(max_x, max_y, max_z);
        point3 v011(min_x, max_y, max_z);

        add_triangle(v100, v000, v110, type, albedo, fuzz, ir);
        add_triangle(v010, v110, v000, type, albedo, fuzz, ir);
        add_triangle(v101, v100, v111, type, albedo, fuzz, ir);
        add_triangle(v110, v111, v100, type, albedo, fuzz, ir);
        add_triangle(v001, v101, v011, type, albedo, fuzz, ir);
        add_triangle(v111, v011, v101, type, albedo, fuzz, ir);
        add_triangle(v000, v001, v010, type, albedo, fuzz, ir);
        add_triangle(v011, v010, v001, type, albedo, fuzz, ir);
        add_triangle(v010, v011, v110, type, albedo, fuzz, ir);
        add_triangle(v111, v110, v011, type, albedo, fuzz, ir);
        add_triangle(v100, v101, v000, type, albedo, fuzz, ir);
        add_triangle(v001, v000, v101, type, albedo, fuzz, ir);
    }

    void build_bvh() {
        bvh_nodes.clear();
        size_t total_prims = spheres.size() + triangles.size();
        if (total_prims == 0) return;

        std::vector<std::pair<int, bool>> items;
        for (size_t i = 0; i < spheres.size(); ++i) items.push_back({static_cast<int>(i), true});
        for (size_t i = 0; i < triangles.size(); ++i) items.push_back({static_cast<int>(i), false});

        root_node_index = build_bvh_recursive(items, 0, items.size());
    }

    bool hit(const ray& r, double t_min, double t_max, SceneHitRecord& rec) const {
        bool hit_anything = false;
        auto closest_so_far = t_max;
        SceneHitRecord temp_rec;

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

        if (root_node_index != -1) {
            int node_stack[64]; 
            int stack_ptr = 0;
            node_stack[stack_ptr++] = root_node_index;

            while (stack_ptr > 0) {
                int curr_idx = node_stack[--stack_ptr];
                const auto& node = bvh_nodes[curr_idx];

                if (!node.box.hit(r, t_min, closest_so_far)) continue;

                if (node.is_leaf) {
                    if (node.is_sphere) {
                        if (hit_sphere_primitive(spheres[node.prim_index], r, t_min, closest_so_far, temp_rec)) {
                            hit_anything = true;
                            closest_so_far = temp_rec.t;
                            rec = temp_rec;
                        }
                    } else {
                        if (hit_triangle_primitive(triangles[node.prim_index], r, t_min, closest_so_far, temp_rec)) {
                            hit_anything = true;
                            closest_so_far = temp_rec.t;
                            rec = temp_rec;
                        }
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
    int build_bvh_recursive(std::vector<std::pair<int, bool>>& items, size_t start, size_t end) {
        int current_node_idx = bvh_nodes.size();
        bvh_nodes.emplace_back(); 

        size_t span = end - start;
        aabb node_box;

        if (span == 1) {
            if (items[start].second) {
                node_box = spheres[items[start].first].bounding_box();
                bvh_nodes[current_node_idx].is_sphere = true;
            } else {
                node_box = triangles[items[start].first].bounding_box();
                bvh_nodes[current_node_idx].is_sphere = false;
            }
            bvh_nodes[current_node_idx].box = node_box;
            bvh_nodes[current_node_idx].prim_index = items[start].first;
            bvh_nodes[current_node_idx].is_leaf = true;
            return current_node_idx;
        }

        if (items[start].second) node_box = spheres[items[start].first].bounding_box();
        else node_box = triangles[items[start].first].bounding_box();

        for (size_t i = start + 1; i < end; ++i) {
            aabb box2 = items[i].second ? spheres[items[i].first].bounding_box() : triangles[items[i].first].bounding_box();
            node_box = surrounding_box(node_box, box2);
        }

        vec3 size = node_box.max_pt - node_box.min_pt;
        int axis = 0;
        if (size.y() > size.x()) axis = 1;
        if (size.z() > size[axis]) axis = 2;

        auto comparator = [this, axis](const std::pair<int, bool>& a, const std::pair<int, bool>& b) {
            double min_a = a.second ? spheres[a.first].bounding_box().min_pt[axis] : triangles[a.first].bounding_box().min_pt[axis];
            double min_b = b.second ? spheres[b.first].bounding_box().min_pt[axis] : triangles[b.first].bounding_box().min_pt[axis];
            return min_a < min_b;
        };

        std::sort(items.begin() + start, items.begin() + end, comparator);

        size_t mid = start + span / 2;
        
        int left_child = build_bvh_recursive(items, start, mid);
        int right_child = build_bvh_recursive(items, mid, end);

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

    bool hit_triangle_primitive(const TrianglePrimitive& tri, const ray& r, double t_min, double t_max, SceneHitRecord& rec) const {
        vec3 edge1 = tri.v1 - tri.v0;
        vec3 edge2 = tri.v2 - tri.v0;
        vec3 h = cross(r.direction(), edge2);
        double a = dot(edge1, h);

        if (a > -1e-8 && a < 1e-8) return false;

        double f = 1.0 / a;
        vec3 s = r.origin() - tri.v0;
        double u = f * dot(s, h);

        if (u < 0.0 || u > 1.0) return false;

        vec3 q = cross(s, edge1);
        double v = f * dot(r.direction(), q);

        if (v < 0.0 || u + v > 1.0) return false;

        double t = f * dot(edge2, q);

        if (t < t_min || t > t_max) return false;

        rec.t = t;
        rec.p = r.at(t);
        rec.set_face_normal(r, tri.normal);
        rec.mat_type = tri.mat_type;
        rec.albedo = tri.albedo;
        rec.fuzz = tri.fuzz;
        rec.ir = tri.ir;

        return true;
    }
};

#endif