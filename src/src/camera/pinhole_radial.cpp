//
// Created by csl on 11/21/22.
//

#include "veta/camera/pinhole_radial.h"

namespace ns_veta {

    // ---------------------------
    // PinholeIntrinsicRadialK1
    // ---------------------------

    Eintrinsic PinholeIntrinsicRadialK1::GetType() const {
        return PINHOLE_CAMERA_RADIA_K1;
    }

    bool PinholeIntrinsicRadialK1::HaveDisto() const {
        return true;
    }

    Vec2 PinholeIntrinsicRadialK1::AddDisto(const Vec2 &p) const {
        const double k1 = params_[0];

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double r_coeff = (1. + k1 * r2);

        return (p * r_coeff);
    }

    Vec2 PinholeIntrinsicRadialK1::RemoveDisto(const Vec2 &p) const {
        // Compute the radius from which the point p comes from thanks to a bisection
        // Minimize disto(radius(p')^2) == actual Squared(radius(p))

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double radius = (r2 == 0) ? 1. : ::sqrt(BisectionRadiusSolve(params_, r2, DistoFunctor) / r2);
        return radius * p;
    }

    std::vector<double> PinholeIntrinsicRadialK1::GetParams() const {
        std::vector<double> params = PinholeIntrinsic::GetParams();
        params.insert(params.end(), std::begin(params_), std::end(params_));
        return params;
    }

    bool PinholeIntrinsicRadialK1::UpdateFromParams(const std::vector<double> &params) {
        if (params.size() == 4) {
            *this = PinholeIntrinsicRadialK1(
                    static_cast<int>(w_), static_cast<int>(h_), params[0], params[1], params[2], params[3]
            );
            return true;
        } else {
            return false;
        }
    }

    std::vector<int>
    PinholeIntrinsicRadialK1::SubsetParameterization(const IntrinsicParamType &parametrization) const {
        std::vector<int> constant_index;
        const int param = static_cast<int>(parametrization);
        if (!(param & (int) IntrinsicParamType::ADJUST_FOCAL_LENGTH)
            || param & (int) IntrinsicParamType::NONE) {
            constant_index.insert(constant_index.end(), 0);
        }
        if (!(param & (int) IntrinsicParamType::ADJUST_PRINCIPAL_POINT)
            || param & (int) IntrinsicParamType::NONE) {
            constant_index.insert(constant_index.end(), {1, 2});
        }
        if (!(param & (int) IntrinsicParamType::ADJUST_DISTORTION)
            || param & (int) IntrinsicParamType::NONE) {
            constant_index.insert(constant_index.end(), 3);
        }
        return constant_index;
    }

    Vec2 PinholeIntrinsicRadialK1::GetUndistoPixel(const Vec2 &p) const {
        return CamToImg(RemoveDisto(ImgToCam(p)));
    }

    Vec2 PinholeIntrinsicRadialK1::GetDistoPixel(const Vec2 &p) const {
        return CamToImg(AddDisto(ImgToCam(p)));
    }

    IntrinsicBase *PinholeIntrinsicRadialK1::Clone() const {
        return new class_type(*this);
    }

    double PinholeIntrinsicRadialK1::DistoFunctor(const std::vector<double> &params, double r2) {
        const double &k1 = params[0];
        return r2 * Square(1. + r2 * k1);
    }

    // ---------------------------
    // PinholeIntrinsicRadialK3
    // ---------------------------

    Eintrinsic PinholeIntrinsicRadialK3::GetType() const {
        return PINHOLE_CAMERA_RADIA_K3;
    }

    bool PinholeIntrinsicRadialK3::HaveDisto() const {
        return true;
    }

    Vec2 PinholeIntrinsicRadialK3::AddDisto(const Vec2 &p) const {
        const double &k1 = params_[0], &k2 = params_[1], &k3 = params_[2];

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double r_coeff = (1. + k1 * r2 + k2 * r4 + k3 * r6);

        return (p * r_coeff);
    }

    Vec2 PinholeIntrinsicRadialK3::RemoveDisto(const Vec2 &p) const {
        // Compute the radius from which the point p comes from thanks to a bisection
        // Minimize disto(radius(p')^2) == actual Squared(radius(p))

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double radius = (r2 == 0) ? 1. : ::sqrt(BisectionRadiusSolve(params_, r2, DistoFunctor) / r2);
        return radius * p;
    }

    std::vector<double> PinholeIntrinsicRadialK3::GetParams() const {
        std::vector<double> params = PinholeIntrinsic::GetParams();
        params.insert(params.end(), std::begin(params_), std::end(params_));
        return params;
    }

    bool PinholeIntrinsicRadialK3::UpdateFromParams(const std::vector<double> &params) {
        if (params.size() == 6) {
            *this = PinholeIntrinsicRadialK3(
                    static_cast<int>(w_), static_cast<int>(h_), params[0], params[1], params[2],
                    params[3], params[4], params[5]
            );
            return true;
        } else {
            return false;
        }
    }

    std::vector<int>
    PinholeIntrinsicRadialK3::SubsetParameterization(const IntrinsicParamType &parametrization) const {
        std::vector<int> constant_index;
        const int param = static_cast<int>(parametrization);
        if (!(param & (int) IntrinsicParamType::ADJUST_FOCAL_LENGTH)
            || param & (int) IntrinsicParamType::NONE) {
            constant_index.insert(constant_index.end(), 0);
        }
        if (!(param & (int) IntrinsicParamType::ADJUST_PRINCIPAL_POINT)
            || param & (int) IntrinsicParamType::NONE) {
            constant_index.insert(constant_index.end(), {1, 2});
        }
        if (!(param & (int) IntrinsicParamType::ADJUST_DISTORTION)
            || param & (int) IntrinsicParamType::NONE) {
            constant_index.insert(constant_index.end(), {3, 4, 5});
        }
        return constant_index;
    }

    Vec2 PinholeIntrinsicRadialK3::GetUndistoPixel(const Vec2 &p) const {
        return CamToImg(RemoveDisto(ImgToCam(p)));
    }

    Vec2 PinholeIntrinsicRadialK3::GetDistoPixel(const Vec2 &p) const {
        return CamToImg(AddDisto(ImgToCam(p)));
    }

    IntrinsicBase *PinholeIntrinsicRadialK3::Clone() const {
        return new class_type(*this);
    }

    double PinholeIntrinsicRadialK3::DistoFunctor(const std::vector<double> &params, double r2) {
        const double &k1 = params[0], &k2 = params[1], &k3 = params[2];
        return r2 * Square(1. + r2 * (k1 + r2 * (k2 + r2 * k3)));
    }
}