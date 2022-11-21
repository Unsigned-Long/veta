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

    const Views &Veta::GetViews() const { return views; }

    const Poses &Veta::GetPoses() const { return poses; }

    const Intrinsics &Veta::GetIntrinsics() const { return intrinsics; }

    const Landmarks &Veta::GetLandmarks() const { return structure; }

    bool ValidIds(const Veta &sfm_data, EVeta flags_part) {
        const bool bCheck_Intrinsic = (flags_part & INTRINSICS) == INTRINSICS;
        const bool bCheck_Extrinsic = (flags_part & EXTRINSICS) == EXTRINSICS;

        std::set<IndexT> set_id_intrinsics; // unique so we can use a set
        transform(sfm_data.GetIntrinsics().cbegin(), sfm_data.GetIntrinsics().cend(),
                  std::inserter(set_id_intrinsics, set_id_intrinsics.begin()), RetrieveKey());

        std::set<IndexT> set_id_extrinsics; // unique so we can use a set
        transform(sfm_data.GetPoses().cbegin(), sfm_data.GetPoses().cend(),
                  std::inserter(set_id_extrinsics, set_id_extrinsics.begin()), RetrieveKey());

        // Collect existing id_intrinsic && id_extrinsic from views
        std::set<IndexT> reallyDefined_id_intrinsics;
        std::set<IndexT> reallyDefined_id_extrinsics;
        for (const auto &view_it: sfm_data.GetViews()) {
            // If a pose is defined, at least the intrinsic must be valid,
            // In order to generate a valid camera.
            const IndexT id_pose = view_it.second->id_pose;
            const IndexT id_intrinsic = view_it.second->id_intrinsic;

            if (set_id_extrinsics.count(id_pose))
                reallyDefined_id_extrinsics.insert(id_pose); //at least it exists

            if (set_id_intrinsics.count(id_intrinsic))
                reallyDefined_id_intrinsics.insert(id_intrinsic); //at least it exists
        }
        // Check if defined intrinsic & extrinsic are at least connected to views
        bool bRet = true;
        if (bCheck_Intrinsic)
            bRet &= set_id_intrinsics.size() == reallyDefined_id_intrinsics.size();

        if (bCheck_Extrinsic)
            bRet &= set_id_extrinsics.size() == reallyDefined_id_extrinsics.size();

        if (!bRet) {
            std::cout
                    << "There is orphan intrinsics data or poses (some pose(s) or intrinsic(s) do not depend on any view)";
        }

        return bRet;
    }

    bool Load(Veta &sfm_data, const std::string &filename, EVeta flags_part) {
        bool bStatus;
        const std::string ext = extension_part(filename);
        if (ext == "json")
            bStatus = Load_Cereal<cereal::JSONInputArchive>(sfm_data, filename, flags_part);
        else if (ext == "bin")
            bStatus = Load_Cereal<cereal::PortableBinaryInputArchive>(sfm_data, filename, flags_part);
        else if (ext == "xml")
            bStatus = Load_Cereal<cereal::XMLInputArchive>(sfm_data, filename, flags_part);
        else {
            std::cerr << "Unknown sfm_data input format: " << filename;
            return false;
        }

        // Assert that loaded intrinsics | extrinsics are linked to valid view
        if (bStatus && (flags_part & VIEWS) == VIEWS &&
            ((flags_part & INTRINSICS) == INTRINSICS || (flags_part & EXTRINSICS) == EXTRINSICS)) {
            return ValidIds(sfm_data, flags_part);
        }
        return bStatus;
    }

    bool Save(const Veta &sfm_data, const std::string &filename, EVeta flags_part) {
        const std::string ext = extension_part(filename);
        if (ext == "json")
            return Save_Cereal<cereal::JSONOutputArchive>(sfm_data, filename, flags_part);
        else if (ext == "bin")
            return Save_Cereal<cereal::PortableBinaryOutputArchive>(sfm_data, filename, flags_part);
        else if (ext == "xml")
            return Save_Cereal<cereal::XMLOutputArchive>(sfm_data, filename, flags_part);
        else {
            std::cerr << "Unknown sfm_data export format: " << filename;
        }
        return false;
    }
}