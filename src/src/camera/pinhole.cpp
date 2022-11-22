//
// Created by csl on 11/21/22.
//

#include <utility>

#include "veta/camera/pinhole.h"

namespace ns_veta {

    PinholeIntrinsic::PinholeIntrinsic(unsigned int w, unsigned int h, double focalLengthPix, double ppx,
                                       double ppy) : IntrinsicBase(w, h) {
        K << focalLengthPix, 0., ppx, 0., focalLengthPix, ppy, 0., 0., 1.;
        KInv = K.inverse();
    }

    PinholeIntrinsic::PinholeIntrinsic(unsigned int w, unsigned int h, Mat3d KMat)
            : IntrinsicBase(w, h), K(std::move(KMat)) {
        K(0, 0) = K(1, 1) = (K(0, 0) + K(1, 1)) / 2.0;
        KInv = K.inverse();
    }

    Eintrinsic PinholeIntrinsic::GetType() const {
        return PINHOLE_CAMERA;
    }

    const Mat3d &PinholeIntrinsic::KMat() const {
        return K;
    }

    const Mat3d &PinholeIntrinsic::KInvMat() const {
        return KInv;
    }

    double PinholeIntrinsic::Focal() const {
        return K(0, 0);
    }

    Vec2d PinholeIntrinsic::PrincipalPoint() const {
        return {K(0, 2), K(1, 2)};
    }

    Mat3Xd PinholeIntrinsic::operator()(const Mat2Xd &points) const {
        return (KInv * points.colwise().homogeneous()).colwise().normalized();
    }

    Vec2d PinholeIntrinsic::CamToImg(const Vec2d &p) const {
        return Focal() * p + PrincipalPoint();
    }

    Vec2d PinholeIntrinsic::ImgToCam(const Vec2d &p) const {
        return (p - PrincipalPoint()) / Focal();
    }

    bool PinholeIntrinsic::HaveDisto() const {
        return false;
    }

    Vec2d PinholeIntrinsic::AddDisto(const Vec2d &p) const {
        return p;
    }

    Vec2d PinholeIntrinsic::RemoveDisto(const Vec2d &p) const {
        return p;
    }

    double PinholeIntrinsic::ImagePlaneToCameraPlaneError(double value) const {
        return value / Focal();
    }

    Mat34 PinholeIntrinsic::GetProjectiveEquivalent(const Pose &pose) const {
        return K * (Mat34() << pose.Rotation(), pose.Translation()).finished();
    }

    std::vector<double> PinholeIntrinsic::GetParams() const {
        return {K(0, 0), K(0, 2), K(1, 2)};
    }

    bool PinholeIntrinsic::UpdateFromParams(const std::vector<double> &params) {
        if (params.size() == 3) {
            *this = PinholeIntrinsic(imgWidth, imgHeight, params[0], params[1], params[2]);
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

    Vec2d PinholeIntrinsic::GetUndistoPixel(const Vec2d &p) const {
        return p;
    }

    Vec2d PinholeIntrinsic::GetDistoPixel(const Vec2d &p) const {
        return p;
    }

    IntrinsicBase *PinholeIntrinsic::Clone() const {
        return new class_type(*this);
    }

    std::shared_ptr<IntrinsicBase>
    PinholeIntrinsic::Create(unsigned int w, unsigned int h, double focalLengthPix, double ppx, double ppy) {
        return std::make_shared<PinholeIntrinsic>(w, h, focalLengthPix, ppx, ppy);
    }

    std::shared_ptr<IntrinsicBase> PinholeIntrinsic::Create(unsigned int w, unsigned int h, const Mat3d &KMat) {
        return std::make_shared<PinholeIntrinsic>(w, h, KMat);
    }
}