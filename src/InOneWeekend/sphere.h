#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"

class sphere : public hittable {
  public:
    sphere(const point3& center, double radius, shared_ptr<material> mat)
      : center(center), side_length(2.0 * std::fmax(0, radius)), mat(mat) {
        // Calculamos los límites del cubo
        double half_side = side_length / 2.0;
        box_min = point3(center.x() - half_side, center.y() - half_side, center.z() - half_side);
        box_max = point3(center.x() + half_side, center.y() + half_side, center.z() + half_side);
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        // Implementación del algoritmo de intersección de rayos con cajas AABB
        auto t_min = ray_t.min;
        auto t_max = ray_t.max;

        for (int a = 0; a < 3; a++) {
            auto invD = 1.0f / r.direction()[a];
            auto t0 = (box_min[a] - r.origin()[a]) * invD;
            auto t1 = (box_max[a] - r.origin()[a]) * invD;
            if (invD < 0.0f)
                std::swap(t0, t1);
            
            t_min = t0 > t_min ? t0 : t_min;
            t_max = t1 < t_max ? t1 : t_max;
            
            if (t_max <= t_min)
                return false;
        }

        rec.t = t_min;
        rec.p = r.at(rec.t);
        
        // Determinar la normal basada en la cara más cercana al punto de impacto
        double dx = std::min(std::abs(rec.p.x() - box_min.x()), std::abs(rec.p.x() - box_max.x()));
        double dy = std::min(std::abs(rec.p.y() - box_min.y()), std::abs(rec.p.y() - box_max.y()));
        double dz = std::min(std::abs(rec.p.z() - box_min.z()), std::abs(rec.p.z() - box_max.z()));
        
        vec3 outward_normal;
        if (dx <= dy && dx <= dz) {
            outward_normal = vec3(rec.p.x() < center.x() ? -1 : 1, 0, 0);
        } else if (dy <= dz) {
            outward_normal = vec3(0, rec.p.y() < center.y() ? -1 : 1, 0);
        } else {
            outward_normal = vec3(0, 0, rec.p.z() < center.z() ? -1 : 1);
        }
        
        rec.set_face_normal(r, outward_normal);
        rec.mat = mat;

        return true;
    }

  private:
    point3 center;
    double side_length;
    point3 box_min;
    point3 box_max;
    shared_ptr<material> mat;
};

#endif