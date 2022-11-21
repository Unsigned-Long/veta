//
// Created by csl on 11/21/22.
//

#include "veta/camera/spherical.h"

namespace ns_veta {

    EINTRINSIC Intrinsic_Spherical::getType() const {
        return CAMERA_SPHERICAL;
    }

    std::vector<double> Intrinsic_Spherical::getParams() const {
        return {};
    }

    bool Intrinsic_Spherical::updateFromParams(const std::vector<double> &params) {
        return true;
    }

    std::vector<int>
    Intrinsic_Spherical::subsetParameterization(const Intrinsic_Parameter_Type &parametrization) const {
        return {};
    }

    Vec2 Intrinsic_Spherical::cam2ima(const Vec2 &p) const {
        const double size(std::max(w(), h()));
        return {p.x() * size + w() / 2.0, p.y() * size + h() / 2.0};
    }

    Vec2 Intrinsic_Spherical::ima2cam(const Vec2 &p) const {
        const double size(std::max(w(), h()));
        return {(p.x() - w() / 2.0) / size, (p.y() - h() / 2.0) / size};
    }

    Mat3X Intrinsic_Spherical::operator()(const Mat2X &points) const {
        Mat3X bearing(3, points.cols());
        for (Mat2X::Index i(0); i < points.cols(); ++i) {
            const Vec2 uv = ima2cam(points.col(i));

            const double lon = uv.x() * 2 * M_PI;
            const double lat = -uv.y() * 2 * M_PI;

            bearing.col(i) << std::cos(lat) * std::sin(lon), -std::sin(lat), std::cos(lat) * std::cos(lon);
        }
        return bearing;
    }

    Vec2 Intrinsic_Spherical::project(const Vec3 &X, bool ignore_distortion) const {
        const double lon = std::atan2(X.x(), X.z()); // Horizontal normalization of the  X-Z component
        const double lat = std::atan2(-X.y(), std::hypot(X.x(), X.z())); // Tilt angle
        // de-normalization (angle to pixel value)
        return cam2ima({lon / (2 * M_PI), -lat / (2 * M_PI)});
    }

    bool Intrinsic_Spherical::have_disto() const { return false; }

    Vec2 Intrinsic_Spherical::add_disto(const Vec2 &p) const { return p; }

    Vec2 Intrinsic_Spherical::remove_disto(const Vec2 &p) const { return p; }

    Vec2 Intrinsic_Spherical::get_ud_pixel(const Vec2 &p) const { return p; }

    Vec2 Intrinsic_Spherical::get_d_pixel(const Vec2 &p) const { return p; }

    double Intrinsic_Spherical::imagePlane_toCameraPlaneError(double value) const { return value / std::max(w_, h_); }

    Mat34 Intrinsic_Spherical::get_projective_equivalent(const Pose &pose) const {
        return HStack(pose.rotation(), pose.translation());
    }

    IntrinsicBase *Intrinsic_Spherical::clone() const {
        return new class_type(*this);
    }
}