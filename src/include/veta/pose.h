//
// Created by csl on 11/21/22.
//

#ifndef VETA_POSE_H
#define VETA_POSE_H


#include "type_def.hpp"

namespace ns_veta {

    /**
    * @brief Defines a pose in 3d space
    * [R|C] t = -RC
    */
    class Pose {
    protected:

        /// Orientation matrix
        Mat3 rotation_;

        /// Center of rotation
        Vec3 center_;

    public:

        /**
        * @brief Constructor
        * @param r Rotation
        * @param c Center
        * @note Default (without args) defines an Identity pose.
        */
        explicit Pose(Mat3 r = Mat3::Identity(), Vec3 c = Vec3::Zero())
                : rotation_(std::move(r)), center_(std::move(c)) {}

        /**
        * @brief Get Rotation matrix
        * @return Rotation matrix
        */
        [[nodiscard]] const Mat3 &rotation() const;

        /**
        * @brief Get Rotation matrix
        * @return Rotation matrix
        */
        Mat3 &rotation();

        /**
        * @brief Get center of rotation
        * @return center of rotation
        */
        [[nodiscard]] const Vec3 &center() const;

        /**
        * @brief Get center of rotation
        * @return Center of rotation
        */
        Vec3 &center();

        /**
        * @brief Get translation vector
        * @return translation vector
        * @note t = -RC
        */
        [[nodiscard]] Vec3 translation() const;


        /**
        * @brief Apply pose
        * @param p Point
        * @return transformed point
        */
        template<typename T>
        typename T::PlainObject operator()(const T &p) const {
            return rotation_ * (p.colwise() - center_);
        }

        // Specialization for Vec3
        typename Vec3::PlainObject operator()(const Vec3 &p) const;


        /**
        * @brief Composition of poses
        * @param P a Pose
        * @return Composition of current pose and parameter pose
        */
        Pose operator*(const Pose &P) const;


        /**
        * @brief Get inverse of the pose
        * @return Inverse of the pose
        */
        [[nodiscard]] Pose inverse() const;

        /**
        * @brief Return the pose as a single Mat34 matrix [R|t]
        * @return The pose as a Mat34 matrix
        */
        [[nodiscard]]   Mat34 asMatrix() const;

        /**
        * Serialization out
        * @param ar Archive
        */
        template<class Archive>
        void save(Archive &ar) const {
            const std::vector<std::vector<double>> mat = {
                    {rotation_(0, 0), rotation_(0, 1), rotation_(0, 2)},
                    {rotation_(1, 0), rotation_(1, 1), rotation_(1, 2)},
                    {rotation_(2, 0), rotation_(2, 1), rotation_(2, 2)}
            };

            ar(cereal::make_nvp("rotation", mat));

            const std::vector<double> vec = {center_(0), center_(1), center_(2)};
            ar(cereal::make_nvp("center", vec));
        }

        /**
        * @brief Serialization in
        * @param ar Archive
        */
        template<class Archive>
        void load(Archive &ar) {
            std::vector<std::vector<double>> mat(3, std::vector<double>(3));
            ar(cereal::make_nvp("rotation", mat));
            // copy back to the rotation
            rotation_.row(0) = Eigen::Map<const Vec3>(&(mat[0][0]));
            rotation_.row(1) = Eigen::Map<const Vec3>(&(mat[1][0]));
            rotation_.row(2) = Eigen::Map<const Vec3>(&(mat[2][0]));

            std::vector<double> vec(3);
            ar(cereal::make_nvp("center", vec));
            center_ = Eigen::Map<const Vec3>(&vec[0]);
        }
    };
}


#endif //VETA_POSE_H
