//
// Created by csl on 11/21/22.
//

#include "veta/veta.h"

namespace ns_veta {

    bool Veta::IsViewWithPoseDefined(const View::Ptr &view) const {
        if (!view) return false;
        return (view->poseId != UndefinedIndexT && poses.find(view->poseId) != poses.end());
    }

    bool Veta::IsViewWithIntrinsicDefined(const View::Ptr &view) const {
        if (!view) return false;
        return (view->intrinsicId != UndefinedIndexT && intrinsics.find(view->intrinsicId) != intrinsics.end());
    }

    std::optional<Pose> Veta::GetViewPose(const View::Ptr &view) const {
        if (!view || view->poseId == UndefinedIndexT) {
            return {};
        }
        if (auto iter = poses.find(view->poseId);iter == poses.cend()) {
            return {};
        } else {
            return iter->second;
        }
    }

    std::optional<IntrinsicBase::Ptr> Veta::GetViewIntrinsic(const View::Ptr &view) const {
        if (!view || view->intrinsicId == UndefinedIndexT) {
            return {};
        }
        if (auto iter = intrinsics.find(view->intrinsicId);iter == intrinsics.cend()) {
            return {};
        } else {
            return iter->second;
        }
    }

    bool Veta::IsViewWithPoseDefined(IndexT viewId) const {
        if (viewId == UndefinedIndexT) {
            return false;
        }
        if (auto iter = views.find(viewId); iter == views.cend()) {
            return false;
        } else {
            return IsViewWithPoseDefined(iter->second);
        }
    }

    bool Veta::IsViewWithIntrinsicDefined(IndexT viewId) const {
        if (viewId == UndefinedIndexT) {
            return false;
        }
        if (auto iter = views.find(viewId); iter == views.cend()) {
            return false;
        } else {
            return IsViewWithIntrinsicDefined(iter->second);
        }
    }

    std::optional<Pose> Veta::GetViewPose(IndexT viewId) const {
        if (viewId == UndefinedIndexT) {
            return {};
        }
        if (auto iter = views.find(viewId); iter == views.cend()) {
            return {};
        } else {
            return GetViewPose(iter->second);
        }
    }

    std::optional<IntrinsicBase::Ptr> Veta::GetViewIntrinsic(IndexT viewId) const {
        if (viewId == UndefinedIndexT) {
            return {};
        }
        if (auto iter = views.find(viewId); iter == views.cend()) {
            return {};
        } else {
            return GetViewIntrinsic(iter->second);
        }
    }

    bool Veta::IsViewWithTimestampDefined(const View::Ptr &view) {
        if (!view) return false;
        return view->timestamp >= 0.0;
    }

    bool Veta::IsViewWithTimestampDefined(IndexT viewId) {
        if (viewId == UndefinedIndexT) {
            return false;
        }
        if (auto iter = views.find(viewId); iter == views.cend()) {
            return false;
        } else {
            return IsViewWithTimestampDefined(iter->second);
        }
    }

    bool ValidIds(const Veta &veta, Veta::Parts flag) {

        std::set<IndexT> intrinsicsIdSet; // unique so we can use a set
        transform(veta.intrinsics.cbegin(), veta.intrinsics.cend(),
                  std::inserter(intrinsicsIdSet, intrinsicsIdSet.begin()), RetrieveKey());

        std::set<IndexT> extrinsicsIdSet; // unique so we can use a set
        transform(veta.poses.cbegin(), veta.poses.cend(),
                  std::inserter(extrinsicsIdSet, extrinsicsIdSet.begin()), RetrieveKey());

        // Collect existing intrinsicId && id_extrinsic from views
        std::set<IndexT> reallyDefinedIntrinsicsIdSet;
        std::set<IndexT> reallyDefinedExtrinsicsIdSet;
        for (const auto &[viewId, view]: veta.views) {
            // If a pose is defined, at least the intrinsic must be valid,
            // In order to generate a valid camera.
            const IndexT poseId = view->poseId;
            const IndexT intrinsicId = view->intrinsicId;

            if (extrinsicsIdSet.count(poseId))
                reallyDefinedExtrinsicsIdSet.insert(poseId); //at least it exists

            if (intrinsicsIdSet.count(intrinsicId))
                reallyDefinedIntrinsicsIdSet.insert(intrinsicId); //at least it exists
        }
        // Check if defined intrinsic & extrinsic are at least connected to views
        bool bRet = true;
        if (Veta::IsPartsWith(Veta::INTRINSICS, flag))
            bRet &= intrinsicsIdSet.size() == reallyDefinedIntrinsicsIdSet.size();

        if (Veta::IsPartsWith(Veta::EXTRINSICS, flag))
            bRet &= extrinsicsIdSet.size() == reallyDefinedExtrinsicsIdSet.size();

        if (!bRet) {
            std::cout << "There is intrinsics data or poses (some pose(s) or intrinsic(s) do not depend on any view)";
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
        if (bStatus && Veta::IsPartsWith(Veta::VIEWS, flag) &&
            (Veta::IsPartsWith(Veta::INTRINSICS, flag) || Veta::IsPartsWith(Veta::EXTRINSICS, flag))) {
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