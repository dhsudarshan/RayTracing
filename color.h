#ifndef COLOR_H
#define COLOR_H

#include "vec3.h"
#include <iostream>

class color_utils {
private:
    static inline double clamp(double x, double min, double max) {
        if (x < min) return min;
        if (x > max) return max;
        return x;
    }

public:
    static void write_color(std::ostream& out, const color& pixel_color, int samples_per_pixel) {
        auto r = pixel_color.x();
        auto g = pixel_color.y();
        auto b = pixel_color.z();

        auto scale = 1.0 / samples_per_pixel;
        r *= scale;
        g *= scale;
        b *= scale;
 
 
        r = std::sqrt(r);
        g = std::sqrt(g);
        b = std::sqrt(b);

        int r_byte = static_cast<int>(256 * clamp(r, 0.0, 0.999));
        int g_byte = static_cast<int>(256 * clamp(g, 0.0, 0.999));
        int b_byte = static_cast<int>(256 * clamp(b, 0.0, 0.999));

        out << r_byte << ' ' << g_byte << ' ' << b_byte << '\n';
    }
};

#endif