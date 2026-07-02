#ifndef AABB_H
#define AABB_H

#include "vec3.h"
#include "ray.h"
//axis aligned bounding boxes
class aabb {
public:
    point3 min_pt, max_pt;

    aabb() {}
    aabb(const point3& min_pt, const point3& max_pt) : min_pt(min_pt), max_pt(max_pt) {}

    bool hit(const ray& r, double t_min, double t_max) const {
        for (int a = 0; a < 3; a++) {
            auto invD = 1.0 / r.direction()[a];
            auto t0 = (min_pt[a] - r.origin()[a]) * invD;
            auto t1 = (max_pt[a] - r.origin()[a]) * invD;
            if (invD < 0.0f)
                std::swap(t0, t1);
            t_min = t0 > t_min ? t0 : t_min;
            t_max = t1 < t_max ? t1 : t_max;
            if (t_max <= t_min)
                return false;
        }
        return true;
    }
};

#endif