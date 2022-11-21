//
// Created by csl on 11/21/22.
//

#include "veta/pose.h"

namespace ns_veta {

    const Mat3d &Pose::Rotation() const {
        return rotation;
    }

    Mat3d &Pose::Rotation() {
        return rotation;
    }

    const Vec3d &Pose::Center() const {
        return center;
    }

    Vec3d &Pose::Center() {
        return center;
    }

    Pose Pose::operator*(const Pose &P) const {
        return Pose{rotation * P.rotation, P.center + P.rotation.transpose() * center};
    }

    Pose Pose::Inverse() const {
        return Pose{rotation.transpose(), -(rotation * center)};
    }

    Mat34 Pose::AsMatrix() const {
        return (Mat34() << rotation, Translation()).finished();
    }

    typename Vec3d::PlainObject Pose::operator()(const Vec3d &p) const {
        return rotation * (p - center);
    }

    Vec3d Pose::Translation() const {
        return -(rotation * center);
    }

}