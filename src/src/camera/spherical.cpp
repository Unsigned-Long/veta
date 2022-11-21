//
// Created by csl on 11/21/22.
//

#include "veta/camera/spherical.h"

namespace ns_veta {

    Eintrinsic IntrinsicSpherical::getType() const {
        return CAMERA_SPHERICAL;
    }

    std::vector<double> IntrinsicSpherical::getParams() const {
        return {};
    }

    bool IntrinsicSpherical::updateFromParams(const std::vector<double> &params) {
        return true;
    }

    std::vector<int>
    IntrinsicSpherical::subsetParameterization(const IntrinsicParameterType &parametrization) const {
        return {};
    }

    Vec2 IntrinsicSpherical::cam2ima(const Vec2 &p) const {
        const double size(std::max(w(), h()));
        return {p.x() * size + w() / 2.0, p.y() * size + h() / 2.0};
    }

    Vec2 IntrinsicSpherical::ima2cam(const Vec2 &p) const {
        const double size(std::max(w(), h()));
        return {(p.x() - w() / 2.0) / size, (p.y() - h() / 2.0) / size};
    }

    Mat3X IntrinsicSpherical::operator()(const Mat2X &points) const {
        Mat3X bearing(3, points.cols());
        for (Mat2X::Index i(0); i < points.cols(); ++i) {
            const Vec2 uv = ima2cam(points.col(i));

            const double lon = uv.x() * 2 * M_PI;
            const double lat = -uv.y() * 2 * M_PI;

            bearing.col(i) << std::cos(lat) * std::sin(lon), -std::sin(lat), std::cos(lat) * std::cos(lon);
        }
        return bearing;
    }

    Vec2 IntrinsicSpherical::project(const Vec3 &X, bool ignore_distortion) const {
        const double lon = std::atan2(X.x(), X.z()); // Horizontal normalization of the  X-Z component
        const double lat = std::atan2(-X.y(), std::hypot(X.x(), X.z())); // Tilt angle
        // de-normalization (angle to pixel value)
        return cam2ima({lon / (2 * M_PI), -lat / (2 * M_PI)});
    }

    bool IntrinsicSpherical::have_disto() const { return false; }

    Vec2 IntrinsicSpherical::add_disto(const Vec2 &p) const { return p; }

    Vec2 IntrinsicSpherical::remove_disto(const Vec2 &p) const { return p; }

    Vec2 IntrinsicSpherical::get_ud_pixel(const Vec2 &p) const { return p; }

    Vec2 IntrinsicSpherical::get_d_pixel(const Vec2 &p) const { return p; }

    double IntrinsicSpherical::imagePlane_toCameraPlaneError(double value) const { return value / std::max(w_, h_); }

    Mat34 IntrinsicSpherical::get_projective_equivalent(const Pose &pose) const {
        return HStack(pose.rotation(), pose.translation());
    }

    IntrinsicBase *IntrinsicSpherical::clone() const {
        return new class_type(*this);
    }
}