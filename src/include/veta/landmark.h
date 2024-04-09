//
// Created by csl on 11/21/22.
//

#ifndef VETA_LANDMARK_H
#define VETA_LANDMARK_H

#include <utility>
#include "type_def.hpp"

namespace ns_veta {
    // Define 3D-2D tracking data: 3D landmark with its 2D observations
    struct Observation {
    public:
        // red, green, blue
        typedef Eigen::Matrix<uint8_t, 3, 1> Color;

    public:
        Observation() : featId(UndefinedIndexT), x(Vec2d::Zero()), color(Color::Zero()) {}

        Observation(Vec2d p, IndexT idFeat, Color c = Color::Zero())
                : x(std::move(p)), featId(idFeat), color(std::move(c)) {}

        Vec2d x;
        IndexT featId;
        // default: black
        Color color;

        // Serialization
        template<class Archive>
        void save(Archive &ar) const {
            ar(cereal::make_nvp("feat_id", featId));
            const std::vector<double> pp{x(0), x(1)};
            ar(cereal::make_nvp("x", pp));
            const std::vector<uint8_t> c{color(0), color(1), color(2)};
            ar(cereal::make_nvp("color", c));
        }

        // Serialization
        template<class Archive>
        void load(Archive &ar) {
            ar(cereal::make_nvp("feat_id", featId));
            std::vector<double> p(2);
            ar(cereal::make_nvp("x", p));
            x = Eigen::Map<const Vec2d>(&p[0]);
            std::vector<uint8_t> c(3);
            ar(cereal::make_nvp("color", c));
            color = Eigen::Map<const Color>(&c[0]);
        }
    };

    // Observations are indexed by their View_id
    using Observations = HashMap<IndexT, Observation>;

    struct Landmark {
        Vec3d X;
        Observations obs;

        Landmark(Vec3d x, Observations obs) : X(std::move(x)), obs(std::move(obs)) {}

        Landmark() = default;

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
            X = Eigen::Map<const Vec3d>(&point[0]);
            ar(cereal::make_nvp("observations", obs));
        }
    };
}

#endif //VETA_LANDMARK_H
