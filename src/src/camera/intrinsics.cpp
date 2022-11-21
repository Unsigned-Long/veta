//
// Created by csl on 11/21/22.
//

#include "veta/camera/intrinsics.h"

namespace ns_veta {

    unsigned int IntrinsicBase::w() const {
        return w_;
    }

    unsigned int IntrinsicBase::h() const {
        return h_;
    }

    Vec2 IntrinsicBase::project(const Vec3 &X, bool ignore_distortion) const {
        if (this->have_disto() && !ignore_distortion) {
            // apply disto & intrinsics
            return this->cam2ima(this->add_disto(X.hnormalized()));
        } else {
            // apply intrinsics
            return this->cam2ima(X.hnormalized());
        }
    }

    Vec2 IntrinsicBase::residual(const Vec3 &X, const Vec2 &x, bool ignore_distortion) const {
        const Vec2 proj = this->project(X, ignore_distortion);
        return x - proj;
    }

    bool IntrinsicBase::have_disto() const {
        return false;
    }

    std::size_t IntrinsicBase::hashValue() const {
        size_t seed = 0;
        hash_combine(seed, static_cast<int>( this->getType()));
        hash_combine(seed, w_);
        hash_combine(seed, h_);
        const std::vector<double> params = this->getParams();
        for (const auto &param: params)
            hash_combine(seed, param);
        return seed;
    }
}