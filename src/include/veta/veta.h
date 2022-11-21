//
// Created by csl on 11/21/22.
//

#ifndef VETA_VETA_H
#define VETA_VETA_H

#include "veta/pose.h"
#include "veta/view.h"
#include "veta/landmark.h"
#include "veta/camera/intrinsics.h"
#include "fstream"

namespace ns_veta {

    // Define a collection of IntrinsicParameter (indexed by View::id_intrinsic)
    using Intrinsics = Hash_Map<IndexT, std::shared_ptr<IntrinsicBase>>;

    // Define a collection of Pose (indexed by View::id_pose)
    using Poses = Hash_Map<IndexT, Pose>;

    // Define a collection of View (indexed by View::id_view)
    using Views = Hash_Map<IndexT, std::shared_ptr<View>>;

    // Define a collection of landmarks are indexed by their TrackId
    using Landmarks = Hash_Map<IndexT, Landmark>;

    // Generic SfM data container
    // Store structure and camera properties:
    struct Veta {

        /// Considered views
        Views views;
        /// Considered poses (indexed by view.id_pose)
        Poses poses;
        /// Considered camera intrinsics (indexed by view.id_intrinsic)
        Intrinsics intrinsics;
        /// Structure (3D points with their 2D observations)
        Landmarks structure;

        // ---------
        // Accessors
        // ---------
        [[nodiscard]] const Views &GetViews() const;

        [[nodiscard]] const Poses &GetPoses() const;

        [[nodiscard]] const Intrinsics &GetIntrinsics() const;

        [[nodiscard]] const Landmarks &GetLandmarks() const;

        // Check if the View have defined intrinsic and pose
        bool IsPoseAndIntrinsicDefined(const View *view) const;

        // Get the pose associated to a view
        Pose GetPoseOrDie(const View *view) const;
    };

    enum EVeta {
        // Note: Use power of two values in order to use bitwise operators.
        VIEWS = 1,
        EXTRINSICS = 2,
        INTRINSICS = 4,
        STRUCTURE = 8,
        CONTROL_POINTS = 16,
        ALL = VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE | CONTROL_POINTS
    };


    template<typename archiveType>
    bool Load_Cereal(Veta &data, const std::string &filename, EVeta flags_part) {
        const bool bBinary = extension_part(filename) == "bin";

        // List which part of the file must be considered
        const bool b_views = (flags_part & VIEWS) == VIEWS;
        const bool b_intrinsics = (flags_part & INTRINSICS) == INTRINSICS;
        const bool b_extrinsics = (flags_part & EXTRINSICS) == EXTRINSICS;
        const bool b_structure = (flags_part & STRUCTURE) == STRUCTURE;
        const bool b_control_point = (flags_part & CONTROL_POINTS) == CONTROL_POINTS;

        //Create the stream and check it is ok
        std::ifstream stream(filename, std::ios::binary | std::ios::in);
        if (!stream)
            return false;

        // Data serialization
        try {
            archiveType archive(stream);

            std::string version;
            archive(cereal::make_nvp("sfm_data_version", version));

            if (b_views) {
                archive(cereal::make_nvp("views", data.views));
            } else if (bBinary) {
                // Binary file requires to read all the member,
                // read in a temporary object since the data is not needed.
                Views views;
                archive(cereal::make_nvp("views", views));
            }
            if (b_intrinsics) {
                archive(cereal::make_nvp("intrinsics", data.intrinsics));
            } else if (bBinary) {
                // Binary file requires to read all the member,
                // read in a temporary object since the data is not needed.
                Intrinsics intrinsics;
                archive(cereal::make_nvp("intrinsics", intrinsics));
            }

            if (b_extrinsics)
                archive(cereal::make_nvp("extrinsics", data.poses));
            else if (bBinary) {
                // Binary file requires to read all the member,
                // read in a temporary object since the data is not needed.
                Poses poses;
                archive(cereal::make_nvp("extrinsics", poses));
            }

            if (b_structure)
                archive(cereal::make_nvp("structure", data.structure));
            else if (bBinary) {
                // Binary file requires to read all the member,
                // read in a temporary object since the data is not needed.
                Landmarks structure;
                archive(cereal::make_nvp("structure", structure));
            }
        }
        catch (const cereal::Exception &e) {
            std::cerr << e.what();
            stream.close();
            return false;
        }
        stream.close();
        return true;
    }

    template<typename archiveType>
    bool Save_Cereal(const Veta &data, const std::string &filename, EVeta flags_part) {
        // List which part of the file must be considered
        const bool b_views = (flags_part & VIEWS) == VIEWS;
        const bool b_intrinsics = (flags_part & INTRINSICS) == INTRINSICS;
        const bool b_extrinsics = (flags_part & EXTRINSICS) == EXTRINSICS;
        const bool b_structure = (flags_part & STRUCTURE) == STRUCTURE;
        const bool b_control_point = (flags_part & CONTROL_POINTS) == CONTROL_POINTS;

        //Create the stream and check it is ok
        std::ofstream stream(filename.c_str(), std::ios::binary | std::ios::out);
        if (!stream) {
            return false;
        }

        // Data serialization
        {
            archiveType archive(stream);
            const std::string version = "0.3";
            archive(cereal::make_nvp("sfm_data_version", version));

            if (b_views)
                archive(cereal::make_nvp("views", data.views));
            else
                archive(cereal::make_nvp("views", Views()));

            if (b_intrinsics)
                archive(cereal::make_nvp("intrinsics", data.intrinsics));
            else
                archive(cereal::make_nvp("intrinsics", Intrinsics()));

            if (b_extrinsics)
                archive(cereal::make_nvp("extrinsics", data.poses));
            else
                archive(cereal::make_nvp("extrinsics", Poses()));

            // Structure -> See for export in another file
            if (b_structure)
                archive(cereal::make_nvp("structure", data.structure));
            else
                archive(cereal::make_nvp("structure", Landmarks()));
        }
        stream.close();
        return true;
    }

    template bool
    Load_Cereal<cereal::BinaryInputArchive>(Veta &data, const std::string &filename, EVeta flags_part);

    template bool
    Load_Cereal<cereal::PortableBinaryInputArchive>(Veta &data, const std::string &filename, EVeta flags_part);

    template bool
    Load_Cereal<cereal::JSONInputArchive>(Veta &data, const std::string &filename, EVeta flags_part);

    template bool
    Load_Cereal<cereal::XMLInputArchive>(Veta &data, const std::string &filename, EVeta flags_part);

    template bool
    Save_Cereal<cereal::BinaryOutputArchive>(const Veta &data, const std::string &filename, EVeta flags_part);

    template bool
    Save_Cereal<cereal::PortableBinaryOutputArchive>(const Veta &data, const std::string &filename, EVeta flags_part);

    template bool
    Save_Cereal<cereal::JSONOutputArchive>(const Veta &data, const std::string &filename, EVeta flags_part);

    template bool
    Save_Cereal<cereal::XMLOutputArchive>(const Veta &data, const std::string &filename, EVeta flags_part);

    ///Check that each pose have a valid intrinsic and pose id in the existing View ids
    static bool ValidIds(const Veta &sfm_data, EVeta flags_part);

    /// Load SfM_Data SfM scene from a file
    static bool Load(Veta &sfm_data, const std::string &filename, EVeta flags_part);

    // Save SfM_Data SfM scene to a file
    static bool Save(const Veta &sfm_data, const std::string &filename, EVeta flags_part);


}


#endif //VETA_VETA_H
