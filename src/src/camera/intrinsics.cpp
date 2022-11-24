//
// Created by csl on 11/21/22.
//

#include "veta/camera/intrinsics.h"

namespace ns_veta {

    unsigned int IntrinsicBase::Width() const {
        return imgWidth;
    }

    unsigned int IntrinsicBase::Height() const {
        return imgHeight;
    }

    Vec2d IntrinsicBase::Project(const Vec3d &X, bool ignoreDisto) const {
        if (this->HaveDisto() && !ignoreDisto) {
            // apply disto & intrinsics
            return this->CamToImg(this->AddDisto(X.hnormalized()));
        } else {
            // apply intrinsics
            return this->CamToImg(X.hnormalized());
        }
    }

    Vec2d IntrinsicBase::Residual(const Vec3d &X, const Vec2d &x, bool ignoreDisto) const {
        const Vec2d proj = this->Project(X, ignoreDisto);
        return x - proj;
    }

    bool IntrinsicBase::HaveDisto() const {
        return false;
    }

    std::size_t IntrinsicBase::HashValue() const {
        size_t seed = 0;
        HashCombine(seed, static_cast<int>( this->GetType()));
        HashCombine(seed, imgWidth);
        HashCombine(seed, imgHeight);
        const std::vector<double> params = this->GetParams();
        for (const auto &param: params)
            HashCombine(seed, param);
        return seed;
    }
}