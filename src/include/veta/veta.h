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

    // Define a collection of IntrinsicParameter (indexed by View::intrinsicId)
    using Intrinsics = HashMap<IndexT, std::shared_ptr<IntrinsicBase>>;

    // Define a collection of Pose (indexed by View::poseId)
    using Poses = HashMap<IndexT, Posed>;

    // Define a collection of View (indexed by View::viewId)
    using Views = HashMap<IndexT, std::shared_ptr<View>>;

    // Define a collection of landmarks are indexed by their TrackId
    using Landmarks = HashMap<IndexT, Landmark>;

    struct IndexGenerator {
    protected:
        static IndexT ViewIdCounter;

        static IndexT PoseIdCounter;

        static IndexT IntrinsicsIdCounter;

        static IndexT LandmarkIdCounter;

        static IndexT FeatureIdCounter;

    public:

        // generate

        static IndexT GenNewViewId();

        static IndexT GenNewPoseId();

        static IndexT GenNewIntrinsicsId();

        static IndexT GenNewLandmarkId();

        static IndexT GenNewFeatureId();

        // reset

        static void ResetViewIdCounter();

        static void ResetPoseIdCounter();

        static void ResetIntrinsicsIdCounter();

        static void ResetLandmarkIdCounter();

        static void ResetFeatureIdCounter();
    };

    // Generic SfM data container
    // Store structure and camera properties:
    struct Veta {
    public:
        enum Parts : int {
            /**
             * @brief options
             */
            NONE = 1 << 0,
            VIEWS = 1 << 1,
            EXTRINSICS = 1 << 2,
            INTRINSICS = 1 << 3,
            STRUCTURE = 1 << 4,
            ALL = VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE
        };

        static bool IsPartsWith(int desired, int curParts) {
            return (desired == (desired & curParts));
        }

        /**
         * @brief override operator '<<' for type 'Parts'
         */
        friend std::ostream &operator<<(std::ostream &os, const Parts &curParts) {
            std::stringstream stream;
            int count = 0;
            if (IsPartsWith(VIEWS, curParts)) {
                stream << "VIEWS";
                ++count;
            }
            if (IsPartsWith(EXTRINSICS, curParts)) {
                stream << " | EXTRINSICS";
                ++count;
            }
            if (IsPartsWith(INTRINSICS, curParts)) {
                stream << " | INTRINSICS";
                ++count;
            }
            if (IsPartsWith(STRUCTURE, curParts)) {
                stream << " | STRUCTURE";
                ++count;
            }
            if (count == 0) {
                os << "NONE";
            } else if (count == 4) {
                os << "ALL";
            } else {
                std::string str = stream.str();
                if (str.at(1) == '|') {
                    str = str.substr(3, str.size() - 3);
                }
                os << str;
            }
            return os;
        };

    public:

        /// Considered views
        Views views;
        /// Considered poses (indexed by view.poseId)
        Poses poses;
        /// Considered camera intrinsics (indexed by view.intrinsicId)
        Intrinsics intrinsics;
        /// Structure (3D points with their 2D observations)
        Landmarks structure;

    public:
        using Ptr = std::shared_ptr<Veta>;

        Veta();

        static Ptr Create();
    };

    template<typename archiveType>
    bool LoadCereal(Veta &data, const std::string &filename, Veta::Parts flag) {
        const bool bBinary = ExtensionPart(filename) == "bin";

        //Create the stream and check it is ok
        std::ifstream stream(filename, std::ios::binary | std::ios::in);
        if (!stream)
            return false;

        // Data serialization
        try {
            archiveType archive(stream);

            std::string version;
            archive(cereal::make_nvp("veta_version", version));

            if (Veta::IsPartsWith(Veta::VIEWS, flag)) {
                archive(cereal::make_nvp("views", data.views));
            } else if (bBinary) {
                // Binary file requires to read all the member,
                // read in a temporary object since the data is not needed.
                Views views;
                archive(cereal::make_nvp("views", views));
            }
            if (Veta::IsPartsWith(Veta::INTRINSICS, flag)) {
                archive(cereal::make_nvp("intrinsics", data.intrinsics));
            } else if (bBinary) {
                // Binary file requires to read all the member,
                // read in a temporary object since the data is not needed.
                Intrinsics intrinsics;
                archive(cereal::make_nvp("intrinsics", intrinsics));
            }

            if (Veta::IsPartsWith(Veta::EXTRINSICS, flag))
                archive(cereal::make_nvp("extrinsics", data.poses));
            else if (bBinary) {
                // Binary file requires to read all the member,
                // read in a temporary object since the data is not needed.
                Poses poses;
                archive(cereal::make_nvp("extrinsics", poses));
            }

            if (Veta::IsPartsWith(Veta::STRUCTURE, flag))
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
    bool SaveCereal(const Veta &data, const std::string &filename, Veta::Parts flag) {

        //Create the stream and check it is ok
        std::ofstream stream(filename.c_str(), std::ios::binary | std::ios::out);
        if (!stream) {
            return false;
        }

        // Data serialization
        {
            archiveType archive(stream);
            const std::string version = "0.1";
            archive(cereal::make_nvp("veta_version", version));

            if (Veta::IsPartsWith(Veta::VIEWS, flag))
                archive(cereal::make_nvp("views", data.views));
            else
                archive(cereal::make_nvp("views", Views()));

            if (Veta::IsPartsWith(Veta::INTRINSICS, flag))
                archive(cereal::make_nvp("intrinsics", data.intrinsics));
            else
                archive(cereal::make_nvp("intrinsics", Intrinsics()));

            if (Veta::IsPartsWith(Veta::EXTRINSICS, flag))
                archive(cereal::make_nvp("extrinsics", data.poses));
            else
                archive(cereal::make_nvp("extrinsics", Poses()));

            // Structure -> See for export in another file
            if (Veta::IsPartsWith(Veta::STRUCTURE, flag))
                archive(cereal::make_nvp("structure", data.structure));
            else
                archive(cereal::make_nvp("structure", Landmarks()));
        }
        stream.close();
        return true;
    }

    template bool
    LoadCereal<cereal::BinaryInputArchive>(Veta &data, const std::string &filename, Veta::Parts flag);

    template bool
    LoadCereal<cereal::PortableBinaryInputArchive>(Veta &data, const std::string &filename, Veta::Parts flag);

    template bool
    LoadCereal<cereal::JSONInputArchive>(Veta &data, const std::string &filename, Veta::Parts flag);

    template bool
    LoadCereal<cereal::XMLInputArchive>(Veta &data, const std::string &filename, Veta::Parts flag);

    template bool
    SaveCereal<cereal::BinaryOutputArchive>(const Veta &data, const std::string &filename, Veta::Parts flag);

    template bool
    SaveCereal<cereal::PortableBinaryOutputArchive>(const Veta &data, const std::string &filename, Veta::Parts flag);

    template bool
    SaveCereal<cereal::JSONOutputArchive>(const Veta &data, const std::string &filename, Veta::Parts flag);

    template bool
    SaveCereal<cereal::XMLOutputArchive>(const Veta &data, const std::string &filename, Veta::Parts flag);

    ///Check that each pose have a valid intrinsic and pose id in the existing View ids
    bool ValidIds(const Veta &veta, Veta::Parts flag);

    /// Load SfM_Data SfM scene from a file
    bool Load(Veta &veta, const std::string &filename, Veta::Parts flag);

    // Save SfM_Data SfM scene to a file
    bool Save(const Veta &veta, const std::string &filename, Veta::Parts flag);


}


#endif //VETA_VETA_H
