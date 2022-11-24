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

    Pose Pose::operator*(const Pose &pose) const {
        return Pose{rotation * pose.rotation, rotation * pose.translation + translation};
    }

    Pose Pose::Inverse() const {
        return Pose{rotation.transpose(), -(rotation.transpose() * translation)};
    }

    Mat34d Pose::AsMatrix() const {
        return (Mat34d() << rotation, translation).finished();
    }

    typename Vec3d::PlainObject Pose::operator()(const Vec3d &p) const {
        return rotation * p + translation;
    }

    const Vec3d &Pose::Translation() const {
        return translation;
    }

    Vec3d &Pose::Translation() {
        return translation;
    }

}