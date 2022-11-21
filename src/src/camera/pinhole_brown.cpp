//
// Created by csl on 11/21/22.
//

#include "veta/camera/pinhole_brown.h"

namespace ns_veta {

    Pinhole_Intrinsic_Brown_T2::Pinhole_Intrinsic_Brown_T2(int w, int h, double focal, double ppx, double ppy,
                                                           double k1, double k2, double k3, double t1, double t2)
            : Pinhole_Intrinsic(w, h, focal, ppx, ppy), params_({k1, k2, k3, t1, t2}) {}

    EINTRINSIC Pinhole_Intrinsic_Brown_T2::getType() const {
        return PINHOLE_CAMERA_BROWN;
    }

    bool Pinhole_Intrinsic_Brown_T2::have_disto() const {
        return true;
    }

    Vec2 Pinhole_Intrinsic_Brown_T2::add_disto(const Vec2 &p) const {
        return (p + distoFunction(params_, p));
    }

    Vec2 Pinhole_Intrinsic_Brown_T2::remove_disto(const Vec2 &p) const {
        const double epsilon = 1e-10; //criteria to stop the iteration
        Vec2 p_u = p;

        Vec2 d = distoFunction(params_, p_u);
        while ((p_u + d - p).lpNorm<1>() > epsilon) //manhattan distance between the two points
        {
            p_u = p - d;
            d = distoFunction(params_, p_u);
        }

        return p_u;
    }

    std::vector<double> Pinhole_Intrinsic_Brown_T2::getParams() const {
        std::vector<double> params = Pinhole_Intrinsic::getParams();
        params.insert(params.end(), std::begin(params_), std::end(params_));
        return params;
    }

    bool Pinhole_Intrinsic_Brown_T2::updateFromParams(const std::vector<double> &params) {
        if (params.size() == 8) {
            *this = Pinhole_Intrinsic_Brown_T2(
                    static_cast<int>(w_), static_cast<int>(h_),
                    params[0], params[1], params[2],
                    params[3], params[4], params[5],
                    params[6], params[7]
            );
            return true;
        } else {
            return false;
        }
    }

    std::vector<int>
    Pinhole_Intrinsic_Brown_T2::subsetParameterization(const Intrinsic_Parameter_Type &parametrization) const {
        std::vector<int> constant_index;
        const int param = static_cast<int>(parametrization);
        if (!(param & (int) Intrinsic_Parameter_Type::ADJUST_FOCAL_LENGTH)
            || param & (int) Intrinsic_Parameter_Type::NONE) {
            constant_index.insert(constant_index.end(), 0);
        }
        if (!(param & (int) Intrinsic_Parameter_Type::ADJUST_PRINCIPAL_POINT)
            || param & (int) Intrinsic_Parameter_Type::NONE) {
            constant_index.insert(constant_index.end(), {1, 2});
        }
        if (!(param & (int) Intrinsic_Parameter_Type::ADJUST_DISTORTION)
            || param & (int) Intrinsic_Parameter_Type::NONE) {
            constant_index.insert(constant_index.end(), {3, 4, 5, 6, 7});
        }
        return constant_index;
    }

    Vec2 Pinhole_Intrinsic_Brown_T2::get_ud_pixel(const Vec2 &p) const {
        return cam2ima(remove_disto(ima2cam(p)));
    }

    Vec2 Pinhole_Intrinsic_Brown_T2::get_d_pixel(const Vec2 &p) const {
        return cam2ima(add_disto(ima2cam(p)));
    }

    IntrinsicBase *Pinhole_Intrinsic_Brown_T2::clone() const {
        return new class_type(*this);
    }

    Vec2 Pinhole_Intrinsic_Brown_T2::distoFunction(const std::vector<double> &params, const Vec2 &p) {
        const double k1 = params[0], k2 = params[1], k3 = params[2], t1 = params[3], t2 = params[4];
        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double k_diff = (k1 * r2 + k2 * r4 + k3 * r6);
        const double t_x = t2 * (r2 + 2 * p(0) * p(0)) + 2 * t1 * p(0) * p(1);
        const double t_y = t1 * (r2 + 2 * p(1) * p(1)) + 2 * t2 * p(0) * p(1);
        return {p(0) * k_diff + t_x, p(1) * k_diff + t_y};
    }
}