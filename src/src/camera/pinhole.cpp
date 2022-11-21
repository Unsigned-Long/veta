//
// Created by csl on 11/21/22.
//

#include "veta/camera/pinhole.h"

namespace ns_veta {

    PinholeIntrinsic::PinholeIntrinsic(unsigned int w, unsigned int h, double focal_length_pix, double ppx,
                                       double ppy) : IntrinsicBase(w, h) {
        K_ << focal_length_pix, 0., ppx, 0., focal_length_pix, ppy, 0., 0., 1.;
        Kinv_ = K_.inverse();
    }

    PinholeIntrinsic::PinholeIntrinsic(unsigned int w, unsigned int h, const Mat3 &K) : IntrinsicBase(w, h), K_(K) {
        K_(0, 0) = K_(1, 1) = (K(0, 0) + K(1, 1)) / 2.0;
        Kinv_ = K_.inverse();
    }

    Eintrinsic PinholeIntrinsic::getType() const {
        return PINHOLE_CAMERA;
    }

    const Mat3 &PinholeIntrinsic::K() const {
        return K_;
    }

    const Mat3 &PinholeIntrinsic::Kinv() const {
        return Kinv_;
    }

    double PinholeIntrinsic::focal() const {
        return K_(0, 0);
    }

    Vec2 PinholeIntrinsic::principal_point() const {
        return {K_(0, 2), K_(1, 2)};
    }

    Mat3X PinholeIntrinsic::operator()(const Mat2X &points) const {
        return (Kinv_ * points.colwise().homogeneous()).colwise().normalized();
    }

    Vec2 PinholeIntrinsic::cam2ima(const Vec2 &p) const {
        return focal() * p + principal_point();
    }

    Vec2 PinholeIntrinsic::ima2cam(const Vec2 &p) const {
        return (p - principal_point()) / focal();
    }

    bool PinholeIntrinsic::have_disto() const {
        return false;
    }

    Vec2 PinholeIntrinsic::add_disto(const Vec2 &p) const {
        return p;
    }

    Vec2 PinholeIntrinsic::remove_disto(const Vec2 &p) const {
        return p;
    }

    double PinholeIntrinsic::imagePlane_toCameraPlaneError(double value) const {
        return value / focal();
    }

    Mat34 PinholeIntrinsic::get_projective_equivalent(const Pose &pose) const {
        return K_ * (Mat34() << pose.rotation(), pose.translation()).finished();
    }

    std::vector<double> PinholeIntrinsic::getParams() const {
        return {K_(0, 0), K_(0, 2), K_(1, 2)};
    }

    bool PinholeIntrinsic::updateFromParams(const std::vector<double> &params) {
        if (params.size() == 3) {
            *this = PinholeIntrinsic(w_, h_, params[0], params[1], params[2]);
            return true;
        } else {
            return false;
        }
    }

    std::vector<int> PinholeIntrinsic::subsetParameterization(const IntrinsicParameterType &parametrization) const {
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
        return constant_index;
    }

    Vec2 PinholeIntrinsic::get_ud_pixel(const Vec2 &p) const {
        return p;
    }

    Vec2 PinholeIntrinsic::get_d_pixel(const Vec2 &p) const {
        return p;
    }

    IntrinsicBase *PinholeIntrinsic::clone() const {
        return new class_type(*this);
    }
}