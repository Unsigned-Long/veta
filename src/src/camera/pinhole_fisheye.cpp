//
// Created by csl on 11/21/22.
//

#include "veta/camera/pinhole_fisheye.h"

namespace ns_veta {

    PinholeIntrinsicFisheye::PinholeIntrinsicFisheye(int w, int h, double focal, double ppx, double ppy, double k1,
                                                     double k2, double k3, double k4)
            : PinholeIntrinsic(w, h, focal, ppx, ppy), params_({k1, k2, k3, k4}) {}

    EINTRINSIC PinholeIntrinsicFisheye::getType() const {
        return PINHOLE_CAMERA_FISHEYE;
    }

    bool PinholeIntrinsicFisheye::have_disto() const {
        return true;
    }

    Vec2 PinholeIntrinsicFisheye::add_disto(const Vec2 &p) const {
        const double eps = 1e-8;
        const double k1 = params_[0], k2 = params_[1], k3 = params_[2], k4 = params_[3];
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

    Vec2 PinholeIntrinsicFisheye::remove_disto(const Vec2 &p) const {
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
                theta = theta_dist / (1 + params_[0] * theta2 + params_[1] * theta4
                                      + params_[2] * theta6 + params_[3] * theta8);
            }
            scale = std::tan(theta) / theta_dist;
        }
        return p * scale;
    }

    std::vector<double> PinholeIntrinsicFisheye::getParams() const {
        std::vector<double> params = PinholeIntrinsic::getParams();
        params.insert(params.end(), std::begin(params_), std::end(params_));
        return params;
    }

    bool PinholeIntrinsicFisheye::updateFromParams(const std::vector<double> &params) {
        if (params.size() == 7) {
            *this = PinholeIntrinsicFisheye(
                    static_cast<int>(w_), static_cast<int>(h_),
                    params[0], params[1], params[2], // focal, ppx, ppy
                    params[3], params[4], params[5], params[6] // k1, k2, k3, k4
            );
            return true;
        } else {
            return false;
        }
    }

    std::vector<int>
    PinholeIntrinsicFisheye::subsetParameterization(const IntrinsicParameterType &parametrization) const {
        std::vector<int> constant_index;
        const int param = static_cast<int>(parametrization);
        if (!(param & (int) IntrinsicParameterType::ADJUST_FOCAL_LENGTH)
            || param & (int) IntrinsicParameterType::NONE) {
            constant_index.insert(constant_index.end(), 0);
        }
        if (!(param & (int) IntrinsicParameterType::ADJUST_PRINCIPAL_POINT)
            || param & (int) IntrinsicParameterType::NONE) {
            constant_index.insert(constant_index.end(), {1, 2});
        }
        if (!(param & (int) IntrinsicParameterType::ADJUST_DISTORTION)
            || param & (int) IntrinsicParameterType::NONE) {
            constant_index.insert(constant_index.end(), {3, 4, 5, 6});
        }
        return constant_index;
    }

    Vec2 PinholeIntrinsicFisheye::get_ud_pixel(const Vec2 &p) const {
        return cam2ima(remove_disto(ima2cam(p)));
    }

    Vec2 PinholeIntrinsicFisheye::get_d_pixel(const Vec2 &p) const {
        return cam2ima(add_disto(ima2cam(p)));
    }

    IntrinsicBase *PinholeIntrinsicFisheye::clone() const {
        return new class_type(*this);
    }
}
