//
// Created by csl on 11/21/22.
//

#include "veta/camera/pinhole_brown.h"

namespace ns_veta {

    PinholeIntrinsicBrownT2::PinholeIntrinsicBrownT2(int w, int h, double fx, double fy, double ppx, double ppy,
                                                     double k1, double k2, double k3, double t1, double t2)
            : PinholeIntrinsic(w, h, fx, fy, ppx, ppy), params({k1, k2, k3, t1, t2}) {}

    Eintrinsic PinholeIntrinsicBrownT2::GetType() const {
        return PINHOLE_CAMERA_BROWN_T2;
    }

    bool PinholeIntrinsicBrownT2::HaveDisto() const {
        return true;
    }

    Vec2d PinholeIntrinsicBrownT2::AddDisto(const Vec2d &p) const {
        return (p + DistoFunction(params, p));
    }

    Vec2d PinholeIntrinsicBrownT2::RemoveDisto(const Vec2d &p) const {
        const double epsilon = 1e-10; //criteria to stop the iteration
        Vec2d p_u = p;

        Vec2d d = DistoFunction(params, p_u);
        while ((p_u + d - p).lpNorm<1>() > epsilon) //manhattan distance between the two points
        {
            p_u = p - d;
            d = DistoFunction(params, p_u);
        }

        return p_u;
    }

    std::vector<double> PinholeIntrinsicBrownT2::GetParams() const {
        std::vector<double> paramsVec = PinholeIntrinsic::GetParams();
        paramsVec.insert(paramsVec.end(), std::begin(params), std::end(params));
        return paramsVec;
    }

    bool PinholeIntrinsicBrownT2::UpdateFromParams(const std::vector<double> &paramsVec) {
        if (paramsVec.size() == 9) {
            *this = PinholeIntrinsicBrownT2(
                    static_cast<int>(imgWidth), static_cast<int>(imgHeight),
                    paramsVec[0], paramsVec[1], paramsVec[2], paramsVec[3], // fx, fy, ppx, ppy
                    paramsVec[4], paramsVec[5], paramsVec[6],               // k1, k2, k3
                    paramsVec[7], paramsVec[8]                              // t1, t2
            );
            return true;
        } else {
            return false;
        }
    }

    std::vector<int>
    PinholeIntrinsicBrownT2::SubsetParameterization(const IntrinsicParamType &parametrization) const {
        std::vector<int> constantIndex;
        const int param = static_cast<int>(parametrization);
        if (!(param & (int) IntrinsicParamType::ADJUST_FOCAL_LENGTH)
            || param & (int) IntrinsicParamType::NONE) {
            constantIndex.insert(constantIndex.end(), {0, 1}); // fx, fy
        }
        if (!(param & (int) IntrinsicParamType::ADJUST_PRINCIPAL_POINT)
            || param & (int) IntrinsicParamType::NONE) {
            constantIndex.insert(constantIndex.end(), {2, 3}); // ppx, ppy
        }
        if (!(param & (int) IntrinsicParamType::ADJUST_DISTORTION)
            || param & (int) IntrinsicParamType::NONE) {
            constantIndex.insert(constantIndex.end(), {4, 5, 6, 7, 8}); // k1, k2, k3, t1, t2
        }
        return constantIndex;
    }

    Vec2d PinholeIntrinsicBrownT2::GetUndistoPixel(const Vec2d &p) const {
        return CamToImg(RemoveDisto(ImgToCam(p)));
    }

    Vec2d PinholeIntrinsicBrownT2::GetDistoPixel(const Vec2d &p) const {
        return CamToImg(AddDisto(ImgToCam(p)));
    }

    IntrinsicBase *PinholeIntrinsicBrownT2::Clone() const {
        return new class_type(*this);
    }

    Vec2d PinholeIntrinsicBrownT2::DistoFunction(const std::vector<double> &params, const Vec2d &p) {
        const double k1 = params[0], k2 = params[1], k3 = params[2], t1 = params[3], t2 = params[4];
        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double k_diff = (k1 * r2 + k2 * r4 + k3 * r6);
        const double t_x = t2 * (r2 + 2 * p(0) * p(0)) + 2 * t1 * p(0) * p(1);
        const double t_y = t1 * (r2 + 2 * p(1) * p(1)) + 2 * t2 * p(0) * p(1);
        return {p(0) * k_diff + t_x, p(1) * k_diff + t_y};
    }

    PinholeIntrinsicBrownT2::Ptr
    PinholeIntrinsicBrownT2::Create(int w, int h, double fx, double fy, double ppx, double ppy,
                                    double k1, double k2, double k3, double t1, double t2) {
        return std::make_shared<PinholeIntrinsicBrownT2>(w, h, fx, fy, ppx, ppy, k1, k2, k3, t1, t2);
    }
}