#ifndef VEC3_H
#define VEC3_H

#include <cmath>
#include <iostream>
#include "rtutils.h"

//vec3 class and some utility functions
class vec3 {
public:
    double e[3];
    //constructors
    vec3() : e{0, 0, 0} {}
    vec3(double e0, double e1, double e2) : e{e0, e1, e2} {}

    double x() const { return e[0]; }
    double y() const { return e[1]; }
    double z() const { return e[2]; }

    //OVERLOADED OPERATORS FOR VECTOR MATHEMATICS
    vec3 operator-() const { return vec3(-e[0], -e[1], -e[2]); } //return -ve vector
    double operator[](int i) const { return e[i]; } //get elements of vector array 
    double& operator[](int i) { return e[i]; } //same as above, just not const function, return reference

    vec3& operator+=(const vec3& v) { //+= operator for vector addition-assignment
        e[0] += v.e[0];
        e[1] += v.e[1];
        e[2] += v.e[2];
        return *this;
    }

    vec3& operator*=(double t) { //for multiplication assignment
        e[0] *= t;
        e[1] *= t;
        e[2] *= t;
        return *this;
    }

    vec3& operator/=(double t) { //division assignment using multiplication defined above
        return *this *= 1.0 / t;
    }

    double length() const { //return magnitude
        return std::sqrt(length_squared());
    }

    double length_squared() const { // square of magnitude(for efficiency)
        return e[0]*e[0] + e[1]*e[1] + e[2]*e[2];
    }
    
    bool near_zero() const { //check if elements too small, minimize floating point errors in zero/null vectors
        const auto s = 1e-8;
        return (std::fabs(e[0]) < s) && (std::fabs(e[1]) < s) && (std::fabs(e[2]) < s);
    }
};
//alias for point and color as they use same structure (x,y,z) and (r,g,b)
using point3 = vec3;   
using color = vec3;    


//overload stream insertion to output vector in desired format
inline std::ostream& operator<<(std::ostream& out, const vec3& v) {
    return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
}

//add using +
inline vec3 operator+(const vec3& u, const vec3& v) {
    return vec3(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
}

//subtract using -
inline vec3 operator-(const vec3& u, const vec3& v) {
    return vec3(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
}

//multiply 2 vectors using *
inline vec3 operator*(const vec3& u, const vec3& v) {
    return vec3(u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]);
}

//multiply by scalar
inline vec3 operator*(double t, const vec3& v) {
    return vec3(t * v.e[0], t * v.e[1], t * v.e[2]);
}

//scalar multiplication, redefinced so change in order they appear doesnt cause errors
inline vec3 operator*(const vec3& v, double t) {
    return t * v;
}

//division by scalar
inline vec3 operator/(const vec3& v, double t) {
    return (1.0 / t) * v;
}

//dot product of 2 vectors
inline double dot(const vec3& u, const vec3& v) {
    return u.e[0] * v.e[0] + u.e[1] * v.e[1] + u.e[2] * v.e[2];
}

//cross product of 2 vectors
inline vec3 cross(const vec3& u, const vec3& v) {
    return vec3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
                u.e[2] * v.e[0] - u.e[0] * v.e[2],
                u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

//unit vector
inline vec3 unit_vector(const vec3& v) {
    return v / v.length();
}

//generate random vector within bounds
inline vec3 random_vec3() {
    return vec3(random_double(), random_double(), random_double());
}

//generate random vector within given min max values
inline vec3 random_vec3(double min, double max) {
    return vec3(random_double(min,max), random_double(min,max), random_double(min,max));
}

//random vector within unit sphere(lies entirely within sphere)
inline vec3 random_in_unit_sphere() {
    while (true) {
        auto p = random_vec3(-1, 1);
        if (p.length_squared() < 1)
            return p;
    }
}

//random unit vector
inline vec3 random_unit_vector() {
    return unit_vector(random_in_unit_sphere());
}

//random vector in hemisphere, basically if it lies in half part of above unit sphere
inline vec3 random_on_hemisphere(const vec3& normal) {
    vec3 on_unit_sphere = random_unit_vector();
    if (dot(on_unit_sphere, normal) > 0.0) 
        return on_unit_sphere;
    else
        return -on_unit_sphere;
}

//ray after reflection represented by vector
inline vec3 reflect(const vec3& v, const vec3& n) {
    return v - 2 * dot(v, n) * n;
}

//ray after refraction represented by vector
inline vec3 refract(const vec3& uv, const vec3& n, double etai_over_etat) { //etai over etat = refractive index
    auto cos_theta = std::min(dot(-uv, n), 1.0);
    vec3 r_out_perp =  etai_over_etat * (uv + cos_theta * n);
    vec3 r_out_parallel = -std::sqrt(std::abs(1.0 - r_out_perp.length_squared())) * n;
    return r_out_perp + r_out_parallel;
}

#endif