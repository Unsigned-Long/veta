//
// Created by csl on 11/21/22.
//

#ifndef VETA_POSE_H
#define VETA_POSE_H


#include "type_def.hpp"

namespace ns_veta {

    /**
    * @brief Defines a pose in 3d space
    */
    template<typename ScaleType>
    class Pose {
    protected:

        // Orientation matrix
        Mat3<ScaleType> rotation;

        // Translation vector
        Vec3<ScaleType> translation;

    public:

        /**
        * @brief Constructor
        * @param r Rotation
        * @param t Translation
        * @note Default (without args) defines an Identity pose.
        */
        explicit Pose(Mat3<ScaleType> r = Mat3<ScaleType>::Identity(), Vec3<ScaleType> t = Vec3<ScaleType>::Zero())
                : rotation(std::move(r)), translation(std::move(t)) {}

        /**
        * @brief Get Rotation matrix
        * @return Rotation matrix
        */
        [[nodiscard]] const Mat3<ScaleType> &Rotation() const {
            return rotation;
        }

        /**
        * @brief Get Rotation matrix
        * @return Rotation matrix
        */
        Mat3<ScaleType> &Rotation() {
            return rotation;
        }

        /**
        * @brief Get Translation vector
        * @return Translation vector
        */
        [[nodiscard]] const Vec3<ScaleType> &Translation() const {
            return translation;
        }

        /**
         * @brief Get Translation vector
         * @return Translation vector
         */
        Vec3<ScaleType> &Translation() {
            return translation;
        }

        /**
        * @brief Apply pose
        * @param p Point
        * @return transformed point
        */
        template<typename T>
        typename T::PlainObject operator()(const T &p) const {
            return rotation * p.colwise() + translation;
        }

        // Specialization for Vec3d
        typename Vec3<ScaleType>::PlainObject operator()(const Vec3<ScaleType> &p) const {
            return rotation * p + translation;
        }

        /**
        * @brief Composition of poses
        * @param pose a Pose
        * @return Composition of current pose and parameter pose
        */
        Pose operator*(const Pose &pose) const {
            return Pose{rotation * pose.rotation, rotation * pose.translation + translation};
        }

        /**
        * @brief Get Inverse of the pose
        * @return Inverse of the pose
        */
        [[nodiscard]] Pose Inverse() const {
            return Pose{rotation.transpose(), -(rotation.transpose() * translation)};
        }

        /**
        * @brief Return the pose as a single Mat34d matrix [R|t]
        * @return The pose as a Mat34d matrix
        */
        [[nodiscard]] Mat34<ScaleType> AsMatrix() const {
            return (Mat34d() << rotation, translation).finished();
        }

        template<typename T>
        Pose<T> Cast() {
            return Pose<T>(rotation.template cast<T>(), translation.template cast<T>());
        }

        /**
        * Serialization out
        * @param ar Archive
        */
        template<class Archive>
        void save(Archive &ar) const {
            const std::vector<std::vector<ScaleType>> mat = {
                    {rotation(0, 0), rotation(0, 1), rotation(0, 2)},
                    {rotation(1, 0), rotation(1, 1), rotation(1, 2)},
                    {rotation(2, 0), rotation(2, 1), rotation(2, 2)}
            };

            ar(cereal::make_nvp("rotation", mat));

            const std::vector<ScaleType> vec = {translation(0), translation(1), translation(2)};
            ar(cereal::make_nvp("translation", vec));
        }

        /**
        * @brief Serialization in
        * @param ar Archive
        */
        template<class Archive>
        void load(Archive &ar) {
            std::vector<std::vector<ScaleType>> mat(3, std::vector<ScaleType>(3));
            ar(cereal::make_nvp("rotation", mat));
            // copy back to the Rotation
            rotation.row(0) = Eigen::Map<const Vec3d>(&(mat[0][0]));
            rotation.row(1) = Eigen::Map<const Vec3d>(&(mat[1][0]));
            rotation.row(2) = Eigen::Map<const Vec3d>(&(mat[2][0]));

            std::vector<ScaleType> vec(3);
            ar(cereal::make_nvp("translation", vec));
            translation = Eigen::Map<const Vec3<ScaleType>>(&vec[0]);
        }
    };

    using Posed = Pose<double>;
    using Posef = Pose<float>;
}


#endif //VETA_POSE_H
