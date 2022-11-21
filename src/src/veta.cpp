//
// Created by csl on 11/21/22.
//

#include "veta/veta.h"

namespace ns_veta {

    bool Veta::IsPoseAndIntrinsicDefined(const View *view) const {
        if (!view) return false;
        return (
                view->id_intrinsic != UndefinedIndexT &&
                view->id_pose != UndefinedIndexT &&
                intrinsics.find(view->id_intrinsic) != intrinsics.end() &&
                poses.find(view->id_pose) != poses.end());
    }

    Pose Veta::GetPoseOrDie(const View *view) const {
        return poses.at(view->id_pose);
    }
}