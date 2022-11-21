//
// Created by csl on 11/21/22.
//

#include "veta/camera/pinhole_eadial.h"

namespace ns_veta {

    // ---------------------------
    // Pinhole_Intrinsic_Radial_K1
    // ---------------------------

    EINTRINSIC Pinhole_Intrinsic_Radial_K1::getType() const {
        return PINHOLE_CAMERA_RADIAL1;
    }

    bool Pinhole_Intrinsic_Radial_K1::have_disto() const {
        return true;
    }

    Vec2 Pinhole_Intrinsic_Radial_K1::add_disto(const Vec2 &p) const {
        const double k1 = params_[0];

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double r_coeff = (1. + k1 * r2);

        return (p * r_coeff);
    }

    Vec2 Pinhole_Intrinsic_Radial_K1::remove_disto(const Vec2 &p) const {
        // Compute the radius from which the point p comes from thanks to a bisection
        // Minimize disto(radius(p')^2) == actual Squared(radius(p))

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double radius = (r2 == 0) ? 1. : ::sqrt(bisection_Radius_Solve(params_, r2, distoFunctor) / r2);
        return radius * p;
    }

    std::vector<double> Pinhole_Intrinsic_Radial_K1::getParams() const {
        std::vector<double> params = Pinhole_Intrinsic::getParams();
        params.insert(params.end(), std::begin(params_), std::end(params_));
        return params;
    }

    bool Pinhole_Intrinsic_Radial_K1::updateFromParams(const std::vector<double> &params) {
        if (params.size() == 4) {
            *this = Pinhole_Intrinsic_Radial_K1(
                    static_cast<int>(w_), static_cast<int>(h_), params[0], params[1], params[2], params[3]
            );
            return true;
        } else {
            return false;
        }
    }

    std::vector<int>
    Pinhole_Intrinsic_Radial_K1::subsetParameterization(const Intrinsic_Parameter_Type &parametrization) const {
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
            constant_index.insert(constant_index.end(), 3);
        }
        return constant_index;
    }

    Vec2 Pinhole_Intrinsic_Radial_K1::get_ud_pixel(const Vec2 &p) const {
        return cam2ima(remove_disto(ima2cam(p)));
    }

    Vec2 Pinhole_Intrinsic_Radial_K1::get_d_pixel(const Vec2 &p) const {
        return cam2ima(add_disto(ima2cam(p)));
    }

    IntrinsicBase *Pinhole_Intrinsic_Radial_K1::clone() const {
        return new class_type(*this);
    }

    double Pinhole_Intrinsic_Radial_K1::distoFunctor(const std::vector<double> &params, double r2) {
        const double &k1 = params[0];
        return r2 * Square(1. + r2 * k1);
    }

    // ---------------------------
    // Pinhole_Intrinsic_Radial_K3
    // ---------------------------

    EINTRINSIC Pinhole_Intrinsic_Radial_K3::getType() const {
        return PINHOLE_CAMERA_RADIAL3;
    }

    bool Pinhole_Intrinsic_Radial_K3::have_disto() const {
        return true;
    }

    Vec2 Pinhole_Intrinsic_Radial_K3::add_disto(const Vec2 &p) const {
        const double &k1 = params_[0], &k2 = params_[1], &k3 = params_[2];

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double r_coeff = (1. + k1 * r2 + k2 * r4 + k3 * r6);

        return (p * r_coeff);
    }

    Vec2 Pinhole_Intrinsic_Radial_K3::remove_disto(const Vec2 &p) const {
        // Compute the radius from which the point p comes from thanks to a bisection
        // Minimize disto(radius(p')^2) == actual Squared(radius(p))

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double radius = (r2 == 0) ? 1. : ::sqrt(bisection_Radius_Solve(params_, r2, distoFunctor) / r2);
        return radius * p;
    }

    std::vector<double> Pinhole_Intrinsic_Radial_K3::getParams() const {
        std::vector<double> params = Pinhole_Intrinsic::getParams();
        params.insert(params.end(), std::begin(params_), std::end(params_));
        return params;
    }

    bool Pinhole_Intrinsic_Radial_K3::updateFromParams(const std::vector<double> &params) {
        if (params.size() == 6) {
            *this = Pinhole_Intrinsic_Radial_K3(
                    static_cast<int>(w_), static_cast<int>(h_), params[0], params[1], params[2],
                    params[3], params[4], params[5]
            );
            return true;
        } else {
            return false;
        }
    }

    std::vector<int>
    Pinhole_Intrinsic_Radial_K3::subsetParameterization(const Intrinsic_Parameter_Type &parametrization) const {
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
            constant_index.insert(constant_index.end(), {3, 4, 5});
        }
        return constant_index;
    }

    Vec2 Pinhole_Intrinsic_Radial_K3::get_ud_pixel(const Vec2 &p) const {
        return cam2ima(remove_disto(ima2cam(p)));
    }

    Vec2 Pinhole_Intrinsic_Radial_K3::get_d_pixel(const Vec2 &p) const {
        return cam2ima(add_disto(ima2cam(p)));
    }

    IntrinsicBase *Pinhole_Intrinsic_Radial_K3::clone() const {
        return new class_type(*this);
    }

    double Pinhole_Intrinsic_Radial_K3::distoFunctor(const std::vector<double> &params, double r2) {
        const double &k1 = params[0], &k2 = params[1], &k3 = params[2];
        return r2 * Square(1. + r2 * (k1 + r2 * (k2 + r2 * k3)));
    }
}