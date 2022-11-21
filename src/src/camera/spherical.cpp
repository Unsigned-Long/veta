//
// Created by csl on 11/21/22.
//

#include "veta/camera/spherical.h"

namespace ns_veta {

    Eintrinsic IntrinsicSpherical::GetType() const {
        return CAMERA_SPHERICAL;
    }

    std::vector<double> IntrinsicSpherical::GetParams() const {
        return {};
    }

    bool IntrinsicSpherical::UpdateFromParams(const std::vector<double> &params) {
        return true;
    }

    std::vector<int>
    IntrinsicSpherical::SubsetParameterization(const IntrinsicParamType &parametrization) const {
        return {};
    }

    Vec2 IntrinsicSpherical::CamToImg(const Vec2 &p) const {
        const double size(std::max(w(), h()));
        return {p.x() * size + w() / 2.0, p.y() * size + h() / 2.0};
    }

    Vec2 IntrinsicSpherical::ImgToCam(const Vec2 &p) const {
        const double size(std::max(w(), h()));
        return {(p.x() - w() / 2.0) / size, (p.y() - h() / 2.0) / size};
    }

    Mat3X IntrinsicSpherical::operator()(const Mat2X &points) const {
        Mat3X bearing(3, points.cols());
        for (Mat2X::Index i(0); i < points.cols(); ++i) {
            const Vec2 uv = ImgToCam(points.col(i));

            const double lon = uv.x() * 2 * M_PI;
            const double lat = -uv.y() * 2 * M_PI;

            bearing.col(i) << std::cos(lat) * std::sin(lon), -std::sin(lat), std::cos(lat) * std::cos(lon);
        }
        return bearing;
    }

    Vec2 IntrinsicSpherical::Project(const Vec3 &X, bool ignore_distortion) const {
        const double lon = std::atan2(X.x(), X.z()); // Horizontal normalization of the  X-Z component
        const double lat = std::atan2(-X.y(), std::hypot(X.x(), X.z())); // Tilt angle
        // de-normalization (angle to pixel value)
        return CamToImg({lon / (2 * M_PI), -lat / (2 * M_PI)});
    }

    bool IntrinsicSpherical::HaveDisto() const { return false; }

    Vec2 IntrinsicSpherical::AddDisto(const Vec2 &p) const { return p; }

    Vec2 IntrinsicSpherical::RemoveDisto(const Vec2 &p) const { return p; }

    Vec2 IntrinsicSpherical::GetUndistoPixel(const Vec2 &p) const { return p; }

    Vec2 IntrinsicSpherical::GetDistoPixel(const Vec2 &p) const { return p; }

    double IntrinsicSpherical::ImagePlaneToCameraPlaneError(double value) const { return value / std::max(w_, h_); }

    Mat34 IntrinsicSpherical::GetProjectiveEquivalent(const Pose &pose) const {
        return HStack(pose.rotation(), pose.translation());
    }

    IntrinsicBase *IntrinsicSpherical::Clone() const {
        return new class_type(*this);
    }
}