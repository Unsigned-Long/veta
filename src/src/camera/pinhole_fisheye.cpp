//
// Created by csl on 11/21/22.
//

#include "veta/camera/pinhole_fisheye.h"

namespace ns_veta {

    PinholeIntrinsicFisheye::PinholeIntrinsicFisheye(int w, int h, double focal, double ppx, double ppy, double k1,
                                                     double k2, double k3, double k4)
            : PinholeIntrinsic(w, h, focal, ppx, ppy), params({k1, k2, k3, k4}) {}

    Eintrinsic PinholeIntrinsicFisheye::GetType() const {
        return PINHOLE_CAMERA_FISHEYE;
    }

    bool PinholeIntrinsicFisheye::HaveDisto() const {
        return true;
    }

    Vec2d PinholeIntrinsicFisheye::AddDisto(const Vec2d &p) const {
        const double eps = 1e-8;
        const double k1 = params[0], k2 = params[1], k3 = params[2], k4 = params[3];
        const double r = std::hypot(p(0), p(1));
        const double theta = std::atan(r);
        const double
                theta2 = theta * theta,
                theta3 = theta2 * theta,
                theta4 = theta2 * theta2,
                theta5 = theta4 * theta,
                theta6 = theta3 * theta3,
                theta7 = theta6 * theta,
                theta8 = theta4 * theta4,
                theta9 = theta8 * theta;
        const double theta_dist = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
        const double inv_r = r > eps ? 1.0 / r : 1.0;
        const double cdist = r > eps ? theta_dist * inv_r : 1.0;
        return p * cdist;
    }

    Vec2d PinholeIntrinsicFisheye::RemoveDisto(const Vec2d &p) const {
        const double eps = 1e-8;
        double scale = 1.0;
        const double theta_dist = std::hypot(p(0), p(1));
        if (theta_dist > eps) {
            double theta = theta_dist;
            for (int j = 0; j < 10; ++j) {
                const double theta2 = theta * theta;
                const double theta4 = theta2 * theta2;
                const double theta6 = theta4 * theta2;
                const double theta8 = theta6 * theta2;
                theta = theta_dist / (1 + params[0] * theta2 + params[1] * theta4
                                      + params[2] * theta6 + params[3] * theta8);
            }
            scale = std::tan(theta) / theta_dist;
        }
        return p * scale;
    }

    std::vector<double> PinholeIntrinsicFisheye::GetParams() const {
        std::vector<double> params = PinholeIntrinsic::GetParams();
        params.insert(params.end(), std::begin(params), std::end(params));
        return params;
    }

    bool PinholeIntrinsicFisheye::UpdateFromParams(const std::vector<double> &params) {
        if (params.size() == 7) {
            *this = PinholeIntrinsicFisheye(
                    static_cast<int>(imgWidth), static_cast<int>(imgHeight),
                    params[0], params[1], params[2], // Focal, ppx, ppy
                    params[3], params[4], params[5], params[6] // k1, k2, k3, k4
            );
            return true;
        } else {
            return false;
        }
    }

    std::vector<int>
    PinholeIntrinsicFisheye::SubsetParameterization(const IntrinsicParamType &parametrization) const {
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
            constant_index.insert(constant_index.end(), {3, 4, 5, 6});
        }
        return constant_index;
    }

    Vec2d PinholeIntrinsicFisheye::GetUndistoPixel(const Vec2d &p) const {
        return CamToImg(RemoveDisto(ImgToCam(p)));
    }

    Vec2d PinholeIntrinsicFisheye::GetDistoPixel(const Vec2d &p) const {
        return CamToImg(AddDisto(ImgToCam(p)));
    }

    IntrinsicBase *PinholeIntrinsicFisheye::Clone() const {
        return new class_type(*this);
    }

    PinholeIntrinsicFisheye::Ptr
    PinholeIntrinsicFisheye::Create(int w, int h, double focal, double ppx, double ppy,
                                    double k1, double k2, double k3, double k4) {
        return std::make_shared<PinholeIntrinsicFisheye>(w, h, focal, ppx, ppy, k1, k2, k3, k4);
    }
}
