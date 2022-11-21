//
// Created by csl on 11/21/22.
//

#ifndef VETA_LANDMARK_H
#define VETA_LANDMARK_H

#include "type_def.hpp"

namespace ns_veta {
    // Define 3D-2D tracking data: 3D landmark with its 2D observations
    struct Observation {
        Observation() : id_feat(UndefinedIndexT) {}

        Observation(Vec2 p, IndexT idFeat) : x(std::move(p)), id_feat(idFeat) {}

        Vec2 x;
        IndexT id_feat;

        // Serialization
        template<class Archive>
        void save(Archive &ar) const {
            ar(cereal::make_nvp("id_feat", id_feat));
            const std::vector<double> pp{x(0), x(1)};
            ar(cereal::make_nvp("x", pp));
        }

        // Serialization
        template<class Archive>
        void load(Archive &ar) {
            ar(cereal::make_nvp("id_feat", id_feat));
            std::vector<double> p(2);
            ar(cereal::make_nvp("x", p));
            x = Eigen::Map<const Vec2>(&p[0]);
        }
    };

    // Observations are indexed by their View_id
    using Observations = Hash_Map<IndexT, Observation>;

    struct Landmark {
        Vec3 X;
        Observations obs;

        // Serialization
        template<class Archive>
        void save(Archive &ar) const {
            const std::vector<double> point{X(0), X(1), X(2)};
            ar(cereal::make_nvp("X", point));
            ar(cereal::make_nvp("observations", obs));
        }


        template<class Archive>
        void load(Archive &ar) {
            std::vector<double> point(3);
            ar(cereal::make_nvp("X", point));
            X = Eigen::Map<const Vec3>(&point[0]);
            ar(cereal::make_nvp("observations", obs));
        }
    };
}

#endif //VETA_LANDMARK_H
