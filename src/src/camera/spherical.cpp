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

    Vec2d IntrinsicSpherical::CamToImg(const Vec2d &p) const {
        const double size(std::max(Width(), Height()));
        return {p.x() * size + Width() / 2.0, p.y() * size + Height() / 2.0};
    }

    Vec2d IntrinsicSpherical::ImgToCam(const Vec2d &p) const {
        const double size(std::max(Width(), Height()));
        return {(p.x() - Width() / 2.0) / size, (p.y() - Height() / 2.0) / size};
    }

    Mat3Xd IntrinsicSpherical::operator()(const Mat2Xd &points) const {
        Mat3Xd bearing(3, points.cols());
        for (Mat2Xd::Index i(0); i < points.cols(); ++i) {
            const Vec2d uv = ImgToCam(points.col(i));

            const double lon = uv.x() * 2 * M_PI;
            const double lat = -uv.y() * 2 * M_PI;

            bearing.col(i) << std::cos(lat) * std::sin(lon), -std::sin(lat), std::cos(lat) * std::cos(lon);
        }
        return bearing;
    }

    Vec2d IntrinsicSpherical::Project(const Vec3d &X, bool ignoreDisto) const {
        const double lon = std::atan2(X.x(), X.z()); // Horizontal normalization of the  X-Z component
        const double lat = std::atan2(-X.y(), std::hypot(X.x(), X.z())); // Tilt angle
        // de-normalization (angle to pixel value)
        return CamToImg({lon / (2 * M_PI), -lat / (2 * M_PI)});
    }

    bool IntrinsicSpherical::HaveDisto() const { return false; }

    Vec2d IntrinsicSpherical::AddDisto(const Vec2d &p) const { return p; }

    Vec2d IntrinsicSpherical::RemoveDisto(const Vec2d &p) const { return p; }

    Vec2d IntrinsicSpherical::GetUndistoPixel(const Vec2d &p) const { return p; }

    Vec2d IntrinsicSpherical::GetDistoPixel(const Vec2d &p) const { return p; }

    double IntrinsicSpherical::ImagePlaneToCameraPlaneError(double value) const {
        return value / std::max(imgWidth, imgHeight);
    }

    Mat34d IntrinsicSpherical::GetProjectiveEquivalent(const Posed &RefToCam) const {
        return HStack(RefToCam.Rotation().matrix(), RefToCam.Translation());
    }

    IntrinsicBase *IntrinsicSpherical::Clone() const {
        return new class_type(*this);
    }

    IntrinsicSpherical::Ptr IntrinsicSpherical::Create(unsigned int w, unsigned int h) {
        return std::make_shared<IntrinsicSpherical>(w, h);
    }
}