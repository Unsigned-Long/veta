//
// Created by csl on 11/21/22.
//

#ifndef VETA_VIEW_H
#define VETA_VIEW_H

#include "veta/type_def.hpp"

namespace ns_veta {

    // A view define an image by a string and unique indexes for the view, the camera intrinsic & the pose
    struct View {

        // id of the view
        IndexT viewId;

        // Index of intrinsics and the pose
        IndexT intrinsicId, poseId;

        // image size
        IndexT imgWidth, imgHeight;

        // Constructor (use unique index for the viewId)
        explicit View(IndexT viewId = UndefinedIndexT, IndexT intrinsicId = UndefinedIndexT,
                      IndexT poseId = UndefinedIndexT, IndexT width = UndefinedIndexT, IndexT height = UndefinedIndexT)
                : viewId(viewId), intrinsicId(intrinsicId), poseId(poseId), imgWidth(width), imgHeight(height) {}

        virtual ~View() = default;

        /**
        * Serialization out
        * @param ar Archive
        */
        template<class Archive>
        void save(Archive &ar) const {
            ar(cereal::make_nvp("width", imgWidth),
               cereal::make_nvp("height", imgHeight),
               cereal::make_nvp("viewId", viewId),
               cereal::make_nvp("intrinsicId", intrinsicId),
               cereal::make_nvp("poseId", poseId));
        }

        /**
        * @brief Serialization in
        * @param ar Archive
        */
        template<class Archive>
        void load(Archive &ar) {
            ar(cereal::make_nvp("width", imgWidth),
               cereal::make_nvp("height", imgHeight),
               cereal::make_nvp("viewId", viewId),
               cereal::make_nvp("intrinsicId", intrinsicId),
               cereal::make_nvp("poseId", poseId));
        }
    };
}


#endif //VETA_VIEW_H
