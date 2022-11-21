//
// Created by csl on 11/21/22.
//

#include "veta/camera/pinhole.h"

namespace ns_veta {

    Pinhole_Intrinsic::Pinhole_Intrinsic(unsigned int w, unsigned int h, double focal_length_pix, double ppx,
                                         double ppy) : IntrinsicBase(w, h) {
        K_ << focal_length_pix, 0., ppx, 0., focal_length_pix, ppy, 0., 0., 1.;
        Kinv_ = K_.inverse();
    }

    Pinhole_Intrinsic::Pinhole_Intrinsic(unsigned int w, unsigned int h, const Mat3 &K) : IntrinsicBase(w, h), K_(K) {
        K_(0, 0) = K_(1, 1) = (K(0, 0) + K(1, 1)) / 2.0;
        Kinv_ = K_.inverse();
    }

    EINTRINSIC Pinhole_Intrinsic::getType() const {
        return PINHOLE_CAMERA;
    }

    const Mat3 &Pinhole_Intrinsic::K() const {
        return K_;
    }

    const Mat3 &Pinhole_Intrinsic::Kinv() const {
        return Kinv_;
    }

    double Pinhole_Intrinsic::focal() const {
        return K_(0, 0);
    }

    Vec2 Pinhole_Intrinsic::principal_point() const {
        return {K_(0, 2), K_(1, 2)};
    }

    Mat3X Pinhole_Intrinsic::operator()(const Mat2X &points) const {
        return (Kinv_ * points.colwise().homogeneous()).colwise().normalized();
    }

    Vec2 Pinhole_Intrinsic::cam2ima(const Vec2 &p) const {
        return focal() * p + principal_point();
    }

    Vec2 Pinhole_Intrinsic::ima2cam(const Vec2 &p) const {
        return (p - principal_point()) / focal();
    }

    bool Pinhole_Intrinsic::have_disto() const {
        return false;
    }

    Vec2 Pinhole_Intrinsic::add_disto(const Vec2 &p) const {
        return p;
    }

    Vec2 Pinhole_Intrinsic::remove_disto(const Vec2 &p) const {
        return p;
    }

    double Pinhole_Intrinsic::imagePlane_toCameraPlaneError(double value) const {
        return value / focal();
    }

    Mat34 Pinhole_Intrinsic::get_projective_equivalent(const Pose &pose) const {
        return K_ * (Mat34() << pose.rotation(), pose.translation()).finished();
    }

    std::vector<double> Pinhole_Intrinsic::getParams() const {
        return {K_(0, 0), K_(0, 2), K_(1, 2)};
    }

    bool Pinhole_Intrinsic::updateFromParams(const std::vector<double> &params) {
        if (params.size() == 3) {
            *this = Pinhole_Intrinsic(w_, h_, params[0], params[1], params[2]);
            return true;
        } else {
            return false;
        }
    }

    std::vector<int> Pinhole_Intrinsic::subsetParameterization(const Intrinsic_Parameter_Type &parametrization) const {
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
        return constant_index;
    }

    Vec2 Pinhole_Intrinsic::get_ud_pixel(const Vec2 &p) const {
        return p;
    }

    Vec2 Pinhole_Intrinsic::get_d_pixel(const Vec2 &p) const {
        return p;
    }

    IntrinsicBase *Pinhole_Intrinsic::clone() const {
        return new class_type(*this);
    }
}