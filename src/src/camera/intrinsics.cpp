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

    Vec2 IntrinsicBase::Project(const Vec3 &X, bool ignore_distortion) const {
        if (this->HaveDisto() && !ignore_distortion) {
            // apply disto & intrinsics
            return this->CamToImg(this->AddDisto(X.hnormalized()));
        } else {
            // apply intrinsics
            return this->CamToImg(X.hnormalized());
        }
    }

    Vec2 IntrinsicBase::Residual(const Vec3 &X, const Vec2 &x, bool ignore_distortion) const {
        const Vec2 proj = this->Project(X, ignore_distortion);
        return x - proj;
    }

    bool IntrinsicBase::HaveDisto() const {
        return false;
    }

    std::size_t IntrinsicBase::HashValue() const {
        size_t seed = 0;
        hash_combine(seed, static_cast<int>( this->GetType()));
        hash_combine(seed, w_);
        hash_combine(seed, h_);
        const std::vector<double> params = this->GetParams();
        for (const auto &param: params)
            hash_combine(seed, param);
        return seed;
    }
}