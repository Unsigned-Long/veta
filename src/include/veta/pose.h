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
        Mat3d rotation_;

        /// Center of Rotation
        Vec3d center_;

    public:

        /**
        * @brief Constructor
        * @param r Rotation
        * @param c Center
        * @note Default (without args) defines an Identity pose.
        */
        explicit Pose(Mat3d r = Mat3d::Identity(), Vec3d c = Vec3d::Zero())
                : rotation_(std::move(r)), center_(std::move(c)) {}

        /**
        * @brief Get Rotation matrix
        * @return Rotation matrix
        */
        [[nodiscard]] const Mat3d &Rotation() const;

        /**
        * @brief Get Rotation matrix
        * @return Rotation matrix
        */
        Mat3d &Rotation();

        /**
        * @brief Get Center of Rotation
        * @return Center of Rotation
        */
        [[nodiscard]] const Vec3d &Center() const;

        /**
        * @brief Get Center of Rotation
        * @return Center of Rotation
        */
        Vec3d &Center();

        /**
        * @brief Get Translation vector
        * @return Translation vector
        * @note t = -RC
        */
        [[nodiscard]] Vec3d Translation() const;


        /**
        * @brief Apply pose
        * @param p Point
        * @return transformed point
        */
        template<typename T>
        typename T::PlainObject operator()(const T &p) const {
            return rotation_ * (p.colwise() - center_);
        }

        // Specialization for Vec3d
        typename Vec3d::PlainObject operator()(const Vec3d &p) const;


        /**
        * @brief Composition of poses
        * @param P a Pose
        * @return Composition of current pose and parameter pose
        */
        Pose operator*(const Pose &P) const;


        /**
        * @brief Get Inverse of the pose
        * @return Inverse of the pose
        */
        [[nodiscard]] Pose Inverse() const;

        /**
        * @brief Return the pose as a single Mat34 matrix [R|t]
        * @return The pose as a Mat34 matrix
        */
        [[nodiscard]]   Mat34 AsMatrix() const;

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

            ar(cereal::make_nvp("Rotation", mat));

            const std::vector<double> vec = {center_(0), center_(1), center_(2)};
            ar(cereal::make_nvp("Center", vec));
        }

        /**
        * @brief Serialization in
        * @param ar Archive
        */
        template<class Archive>
        void load(Archive &ar) {
            std::vector<std::vector<double>> mat(3, std::vector<double>(3));
            ar(cereal::make_nvp("Rotation", mat));
            // copy back to the Rotation
            rotation_.row(0) = Eigen::Map<const Vec3d>(&(mat[0][0]));
            rotation_.row(1) = Eigen::Map<const Vec3d>(&(mat[1][0]));
            rotation_.row(2) = Eigen::Map<const Vec3d>(&(mat[2][0]));

            std::vector<double> vec(3);
            ar(cereal::make_nvp("Center", vec));
            center_ = Eigen::Map<const Vec3d>(&vec[0]);
        }
    };
}


#endif //VETA_POSE_H
