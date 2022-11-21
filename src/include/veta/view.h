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
        IndexT id_view;

        // Index of intrinsics and the pose
        IndexT id_intrinsic, id_pose;

        // image size
        IndexT ui_width, ui_height;

        // Constructor (use unique index for the view_id)
        explicit View(IndexT view_id = UndefinedIndexT, IndexT intrinsic_id = UndefinedIndexT,
                      IndexT pose_id = UndefinedIndexT, IndexT width = UndefinedIndexT, IndexT height = UndefinedIndexT)
                : id_view(view_id), id_intrinsic(intrinsic_id), id_pose(pose_id), ui_width(width), ui_height(height) {}

        virtual ~View() = default;

        /**
        * Serialization out
        * @param ar Archive
        */
        template<class Archive>
        void save(Archive &ar) const {
            ar(cereal::make_nvp("width", ui_width),
               cereal::make_nvp("height", ui_height),
               cereal::make_nvp("id_view", id_view),
               cereal::make_nvp("id_intrinsic", id_intrinsic),
               cereal::make_nvp("id_pose", id_pose));
        }

        /**
        * @brief Serialization in
        * @param ar Archive
        */
        template<class Archive>
        void load(Archive &ar) {
            ar(cereal::make_nvp("width", ui_width),
               cereal::make_nvp("height", ui_height),
               cereal::make_nvp("id_view", id_view),
               cereal::make_nvp("id_intrinsic", id_intrinsic),
               cereal::make_nvp("id_pose", id_pose));
        }
    };
}


#endif //VETA_VIEW_H
