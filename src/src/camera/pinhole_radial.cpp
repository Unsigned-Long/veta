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

    Vec2d PinholeIntrinsicRadialK1::AddDisto(const Vec2d &p) const {
        const double k1 = params[0];

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double r_coeff = (1. + k1 * r2);

        return (p * r_coeff);
    }

    Vec2d PinholeIntrinsicRadialK1::RemoveDisto(const Vec2d &p) const {
        // Compute the radius from which the point p comes from thanks to a bisection
        // Minimize disto(radius(p')^2) == actual Squared(radius(p))

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double radius = (r2 == 0) ? 1. : ::sqrt(BisectionRadiusSolve(params, r2, DistoFunctor) / r2);
        return radius * p;
    }

    std::vector<double> PinholeIntrinsicRadialK1::GetParams() const {
        std::vector<double> paramsVec = PinholeIntrinsic::GetParams();
        paramsVec.insert(paramsVec.end(), std::begin(paramsVec), std::end(paramsVec));
        return paramsVec;
    }

    bool PinholeIntrinsicRadialK1::UpdateFromParams(const std::vector<double> &paramsVec) {
        if (paramsVec.size() == 4) {
            *this = PinholeIntrinsicRadialK1(
                    static_cast<int>(imgWidth), static_cast<int>(imgHeight),
                    paramsVec[0], paramsVec[1], paramsVec[2], paramsVec[3]
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

    Vec2d PinholeIntrinsicRadialK1::GetUndistoPixel(const Vec2d &p) const {
        return CamToImg(RemoveDisto(ImgToCam(p)));
    }

    Vec2d PinholeIntrinsicRadialK1::GetDistoPixel(const Vec2d &p) const {
        return CamToImg(AddDisto(ImgToCam(p)));
    }

    IntrinsicBase *PinholeIntrinsicRadialK1::Clone() const {
        return new class_type(*this);
    }

    double PinholeIntrinsicRadialK1::DistoFunctor(const std::vector<double> &params, double r2) {
        const double &k1 = params[0];
        return r2 * Square(1. + r2 * k1);
    }

    std::shared_ptr<PinholeIntrinsicRadialK1>
    PinholeIntrinsicRadialK1::Create(int w, int h, double focal, double ppx, double ppy, double k1) {
        return std::make_shared<PinholeIntrinsicRadialK1>(w, h, focal, ppx, ppy, k1);
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

    Vec2d PinholeIntrinsicRadialK3::AddDisto(const Vec2d &p) const {
        const double &k1 = params[0], &k2 = params[1], &k3 = params[2];

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double r_coeff = (1. + k1 * r2 + k2 * r4 + k3 * r6);

        return (p * r_coeff);
    }

    Vec2d PinholeIntrinsicRadialK3::RemoveDisto(const Vec2d &p) const {
        // Compute the radius from which the point p comes from thanks to a bisection
        // Minimize disto(radius(p')^2) == actual Squared(radius(p))

        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double radius = (r2 == 0) ? 1. : ::sqrt(BisectionRadiusSolve(params, r2, DistoFunctor) / r2);
        return radius * p;
    }

    std::vector<double> PinholeIntrinsicRadialK3::GetParams() const {
        std::vector<double> paramsVec = PinholeIntrinsic::GetParams();
        paramsVec.insert(paramsVec.end(), std::begin(paramsVec), std::end(paramsVec));
        return paramsVec;
    }

    bool PinholeIntrinsicRadialK3::UpdateFromParams(const std::vector<double> &paramsVec) {
        if (paramsVec.size() == 6) {
            *this = PinholeIntrinsicRadialK3(
                    static_cast<int>(imgWidth), static_cast<int>(imgHeight), paramsVec[0], paramsVec[1], paramsVec[2],
                    paramsVec[3], paramsVec[4], paramsVec[5]
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

    Vec2d PinholeIntrinsicRadialK3::GetUndistoPixel(const Vec2d &p) const {
        return CamToImg(RemoveDisto(ImgToCam(p)));
    }

    Vec2d PinholeIntrinsicRadialK3::GetDistoPixel(const Vec2d &p) const {
        return CamToImg(AddDisto(ImgToCam(p)));
    }

    IntrinsicBase *PinholeIntrinsicRadialK3::Clone() const {
        return new class_type(*this);
    }

    double PinholeIntrinsicRadialK3::DistoFunctor(const std::vector<double> &params, double r2) {
        const double &k1 = params[0], &k2 = params[1], &k3 = params[2];
        return r2 * Square(1. + r2 * (k1 + r2 * (k2 + r2 * k3)));
    }

    std::shared_ptr<PinholeIntrinsicRadialK3>
    PinholeIntrinsicRadialK3::Create(int w, int h, double focal, double ppx, double ppy, double k1, double k2,
                                     double k3) {
        return std::make_shared<PinholeIntrinsicRadialK3>(w, h, focal, ppx, ppy, k1, k2, k3);
    }
}