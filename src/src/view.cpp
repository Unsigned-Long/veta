//
// Created by csl on 11/21/22.
//

#include "veta/view.h"

namespace ns_veta {

    View::Ptr
    View::Create(TimeT timestamp, IndexT viewId, IndexT intrinsicId, IndexT poseId, IndexT width, IndexT height) {
        return std::make_shared<View>(timestamp, viewId, intrinsicId, poseId, width, height);
    }
}