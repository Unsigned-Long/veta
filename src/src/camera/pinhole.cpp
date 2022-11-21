//
// Created by csl on 11/21/22.
//

#include "veta/camera/pinhole.h"

namespace ns_veta {

    PinholeIntrinsic::PinholeIntrinsic(unsigned int w, unsigned int h, double focal_length_pix, double ppx,
                                       double ppy) : IntrinsicBase(w, h) {
        K_ << focal_length_pix, 0., ppx, 0., focal_length_pix, ppy, 0., 0., 1.;
        KInv_ = K_.inverse();
    }

    PinholeIntrinsic::PinholeIntrinsic(unsigned int w, unsigned int h, const Mat3 &K) : IntrinsicBase(w, h), K_(K) {
        K_(0, 0) = K_(1, 1) = (K(0, 0) + K(1, 1)) / 2.0;
        KInv_ = K_.inverse();
    }

    Eintrinsic PinholeIntrinsic::GetType() const {
        return PINHOLE_CAMERA;
    }

    const Mat3 &PinholeIntrinsic::K() const {
        return K_;
    }

    const Mat3 &PinholeIntrinsic::KInv() const {
        return KInv_;
    }

    double PinholeIntrinsic::Focal() const {
        return K_(0, 0);
    }

    Vec2 PinholeIntrinsic::PrincipalPoint() const {
        return {K_(0, 2), K_(1, 2)};
    }

    Mat3X PinholeIntrinsic::operator()(const Mat2X &points) const {
        return (KInv_ * points.colwise().homogeneous()).colwise().normalized();
    }

    Vec2 PinholeIntrinsic::CamToImg(const Vec2 &p) const {
        return Focal() * p + PrincipalPoint();
    }

    Vec2 PinholeIntrinsic::ImgToCam(const Vec2 &p) const {
        return (p - PrincipalPoint()) / Focal();
    }

    bool PinholeIntrinsic::HaveDisto() const {
        return false;
    }

    Vec2 PinholeIntrinsic::AddDisto(const Vec2 &p) const {
        return p;
    }

    Vec2 PinholeIntrinsic::RemoveDisto(const Vec2 &p) const {
        return p;
    }

    double PinholeIntrinsic::ImagePlaneToCameraPlaneError(double value) const {
        return value / Focal();
    }

    Mat34 PinholeIntrinsic::GetProjectiveEquivalent(const Pose &pose) const {
        return K_ * (Mat34() << pose.rotation(), pose.translation()).finished();
    }

    std::vector<double> PinholeIntrinsic::GetParams() const {
        return {K_(0, 0), K_(0, 2), K_(1, 2)};
    }

    bool PinholeIntrinsic::UpdateFromParams(const std::vector<double> &params) {
        if (params.size() == 3) {
            *this = PinholeIntrinsic(w_, h_, params[0], params[1], params[2]);
            return true;
        } else {
            return false;
        }
    }

    std::vector<int> PinholeIntrinsic::SubsetParameterization(const IntrinsicParamType &parametrization) const {
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
        return constant_index;
    }

    Vec2 PinholeIntrinsic::GetUndistoPixel(const Vec2 &p) const {
        return p;
    }

    Vec2 PinholeIntrinsic::GetDistoPixel(const Vec2 &p) const {
        return p;
    }

    IntrinsicBase *PinholeIntrinsic::Clone() const {
        return new class_type(*this);
    }
}