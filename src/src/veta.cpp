//
// Created by csl on 11/21/22.
//

#include "veta/veta.h"

namespace ns_veta {

    bool Veta::IsPoseAndIntrinsicDefined(const View *view) const {
        if (!view) return false;
        return (view->intrinsicId != UndefinedIndexT &&
                view->poseId != UndefinedIndexT &&
                intrinsics.find(view->intrinsicId) != intrinsics.end() &&
                poses.find(view->poseId) != poses.end());
    }

    Pose Veta::GetPoseOrDie(const View *view) const {
        return poses.at(view->poseId);
    }

    const Views &Veta::GetViews() const { return views; }

    const Poses &Veta::GetPoses() const { return poses; }

    const Intrinsics &Veta::GetIntrinsics() const { return intrinsics; }

    const Landmarks &Veta::GetLandmarks() const { return structure; }

    bool ValidIds(const Veta &veta, Veta::Parts flag) {

        std::set<IndexT> set_id_intrinsics; // unique so we can use a set
        transform(veta.GetIntrinsics().cbegin(), veta.GetIntrinsics().cend(),
                  std::inserter(set_id_intrinsics, set_id_intrinsics.begin()), RetrieveKey());

        std::set<IndexT> set_id_extrinsics; // unique so we can use a set
        transform(veta.GetPoses().cbegin(), veta.GetPoses().cend(),
                  std::inserter(set_id_extrinsics, set_id_extrinsics.begin()), RetrieveKey());

        // Collect existing intrinsicId && id_extrinsic from views
        std::set<IndexT> reallyDefined_id_intrinsics;
        std::set<IndexT> reallyDefined_id_extrinsics;
        for (const auto &view_it: veta.GetViews()) {
            // If a pose is defined, at least the intrinsic must be valid,
            // In order to generate a valid camera.
            const IndexT id_pose = view_it.second->poseId;
            const IndexT id_intrinsic = view_it.second->intrinsicId;

            if (set_id_extrinsics.count(id_pose))
                reallyDefined_id_extrinsics.insert(id_pose); //at least it exists

            if (set_id_intrinsics.count(id_intrinsic))
                reallyDefined_id_intrinsics.insert(id_intrinsic); //at least it exists
        }
        // Check if defined intrinsic & extrinsic are at least connected to views
        bool bRet = true;
        if (Veta::IsPartsWith(Veta::INTRINSICS, flag))
            bRet &= set_id_intrinsics.size() == reallyDefined_id_intrinsics.size();

        if (Veta::IsPartsWith(Veta::EXTRINSICS, flag))
            bRet &= set_id_extrinsics.size() == reallyDefined_id_extrinsics.size();

        if (!bRet) {
            std::cout
                    << "There is orphan intrinsics data or poses (some pose(s) or intrinsic(s) do not depend on any view)";
        }

        return bRet;
    }

    bool Load(Veta &veta, const std::string &filename, Veta::Parts flag) {
        bool bStatus;
        const std::string ext = ExtensionPart(filename);
        if (ext == "json")
            bStatus = LoadCereal<cereal::JSONInputArchive>(veta, filename, flag);
        else if (ext == "bin")
            bStatus = LoadCereal<cereal::PortableBinaryInputArchive>(veta, filename, flag);
        else if (ext == "xml")
            bStatus = LoadCereal<cereal::XMLInputArchive>(veta, filename, flag);
        else {
            std::cerr << "Unknown veta input format: " << filename;
            return false;
        }

        // Assert that loaded intrinsics | extrinsics are linked to valid view
        if (bStatus && (flag & Veta::VIEWS) == Veta::VIEWS &&
            ((flag & Veta::INTRINSICS) == Veta::INTRINSICS || (flag & Veta::EXTRINSICS) == Veta::EXTRINSICS)) {
            return ValidIds(veta, flag);
        }
        return bStatus;
    }

    bool Save(const Veta &veta, const std::string &filename, Veta::Parts flag) {
        const std::string ext = ExtensionPart(filename);
        if (ext == "json")
            return SaveCereal<cereal::JSONOutputArchive>(veta, filename, flag);
        else if (ext == "bin")
            return SaveCereal<cereal::PortableBinaryOutputArchive>(veta, filename, flag);
        else if (ext == "xml")
            return SaveCereal<cereal::XMLOutputArchive>(veta, filename, flag);
        else {
            std::cerr << "Unknown veta export format: " << filename;
        }
        return false;
    }
}