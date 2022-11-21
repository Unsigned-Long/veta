//
// Created by csl on 11/21/22.
//

#include "veta/pose.h"

namespace ns_veta {

    const Mat3 &Pose::rotation() const {
        return rotation_;
    }

    Mat3 &Pose::rotation() {
        return rotation_;
    }

    const Vec3 &Pose::center() const {
        return center_;
    }

    Vec3 &Pose::center() {
        return center_;
    }

    Pose Pose::operator*(const Pose &P) const {
        return Pose{rotation_ * P.rotation_, P.center_ + P.rotation_.transpose() * center_};
    }

    Pose Pose::inverse() const {
        return Pose{rotation_.transpose(), -(rotation_ * center_)};
    }

    Mat34 Pose::asMatrix() const {
        return (Mat34() << rotation_, translation()).finished();
    }

    typename Vec3::PlainObject Pose::operator()(const Vec3 &p) const {
        return rotation_ * (p - center_);
    }

    Vec3 Pose::translation() const {
        return -(rotation_ * center_);
    }

}