//
// Created by csl on 11/21/22.
//

#include "veta/pose.h"

namespace ns_veta {

    const Mat3d &Pose::Rotation() const {
        return rotation_;
    }

    Mat3d &Pose::Rotation() {
        return rotation_;
    }

    const Vec3d &Pose::Center() const {
        return center_;
    }

    Vec3d &Pose::Center() {
        return center_;
    }

    Pose Pose::operator*(const Pose &P) const {
        return Pose{rotation_ * P.rotation_, P.center_ + P.rotation_.transpose() * center_};
    }

    Pose Pose::Inverse() const {
        return Pose{rotation_.transpose(), -(rotation_ * center_)};
    }

    Mat34 Pose::AsMatrix() const {
        return (Mat34() << rotation_, Translation()).finished();
    }

    typename Vec3d::PlainObject Pose::operator()(const Vec3d &p) const {
        return rotation_ * (p - center_);
    }

    Vec3d Pose::Translation() const {
        return -(rotation_ * center_);
    }

}