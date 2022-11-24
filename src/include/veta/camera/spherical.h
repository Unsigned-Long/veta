//
// Created by csl on 11/21/22.
//

#ifndef VETA_SPHERICAL_H
#define VETA_SPHERICAL_H

#include "veta/camera/intrinsics.h"

namespace ns_veta {
/**
 * @brief Implement a Spherical camera model
 */
    class IntrinsicSpherical : public IntrinsicBase {
    public:
        using Ptr = std::shared_ptr<IntrinsicSpherical>;

        using class_type = IntrinsicSpherical;

    public:

        /**
        * @brief Constructor
        * @param w Width of the image plane
        * @param h Height of the image plane
        */
        explicit IntrinsicSpherical(unsigned int w, unsigned int h)
                : IntrinsicBase(w, h) {}

        IntrinsicSpherical() = default;

        static Ptr Create(unsigned int w, unsigned int h);

        ~IntrinsicSpherical() override = default;

        /**
        * @brief Tell from which type the embed camera is
        * @retval CAMERA_SPHERICAL
        */
        [[nodiscard]] Eintrinsic GetType() const override;

        /**
        * @brief Data wrapper for non linear optimization (get data)
        * @return an empty vector of parameter since a spherical camera does not have any intrinsic parameter
        */
        [[nodiscard]] std::vector<double> GetParams() const override;

        /**
        * @brief Data wrapper for non linear optimization (update from data)
        * @param params List of params used to update this intrinsic
        * @retval true if update is correct
        * @retval false if there was an error during update
        */
        bool UpdateFromParams(const std::vector<double> &params) override;

        /**
        * @brief Return the list of parameter indexes that must be held constant
        * @param parametrization The given parametrization
        */
        [[nodiscard]] std::vector<int>
        SubsetParameterization(const IntrinsicParamType &parametrization) const override;

        /**
        * @brief Transform a point from the camera plane to the image plane
        * @param p Camera plane point
        * @return Point on image plane
        */
        [[nodiscard]] Vec2d CamToImg(const Vec2d &p) const override;

        /**
        * @brief Transform a point from the image plane to the camera plane
        * @param p Image plane point
        * @return camera plane point
        */
        [[nodiscard]] Vec2d ImgToCam(const Vec2d &p) const override;

        /**
        * @brief Get bearing vectors from image coordinates
        * @return bearing vectors
        */
        Mat3Xd operator()(const Mat2Xd &points) const override;

        /**
        * @brief Compute projection of a 3D point into the image plane
        * (Apply disto (if any) and Intrinsics)
        * @param pt3D 3D-point to Project on image plane
        * @return Projected (2D) point on image plane
        */
        [[nodiscard]] Vec2d Project(const Vec3d &X, bool ignoreDisto) const override;

        /**
        * @brief Does the camera model handle a distortion field?
        * @retval false
        */
        [[nodiscard]] bool HaveDisto() const override;

        /**
        * @brief Add the distortion field to a point (that is in normalized camera frame)
        * @param p Point before distortion computation (in normalized camera frame)
        * @return the initial point p (spherical camera does not have distortion field)
        */
        [[nodiscard]] Vec2d AddDisto(const Vec2d &p) const override;

        /**
        * @brief Remove the distortion to a camera point (that is in normalized camera frame)
        * @param p Point with distortion
        * @return the initial point p (spherical camera does not have distortion field)
        */
        [[nodiscard]] Vec2d RemoveDisto(const Vec2d &p) const override;

        /**
        * @brief Return the un-distorted pixel (with removed distortion)
        * @param p Input distorted pixel
        * @return Point without distortion
        */
        [[nodiscard]] Vec2d GetUndistoPixel(const Vec2d &p) const override;

        /**
        * @brief Return the distorted pixel (with added distortion)
        * @param p Input pixel
        * @return Distorted pixel
        */
        [[nodiscard]] Vec2d GetDistoPixel(const Vec2d &p) const override;

        /**
        * @brief Normalize a given unit pixel error to the camera plane
        * @param value Error in image plane
        * @return error of passing from the image plane to the camera plane
        */
        [[nodiscard]] double ImagePlaneToCameraPlaneError(double value) const override;

        /**
        * @brief Return the projection matrix (interior & exterior) as a simplified projective projection
        * @param RefToCam Extrinsic matrix
        * @return Concatenation of intrinsic matrix and extrinsic matrix
        */
        [[nodiscard]] Mat34d GetProjectiveEquivalent(const Posed &RefToCam) const override;

        /**
        * @brief Serialization out
        * @param ar Archive
        */
        template<class Archive>
        inline void save(Archive &ar) const {
            ar(cereal::base_class<IntrinsicBase>(this));
        }

        /**
        * @brief  Serialization in
        * @param ar Archive
        */
        template<class Archive>
        inline void load(Archive &ar) {
            ar(cereal::base_class<IntrinsicBase>(this));
        }

        /**
        * @brief Clone the object
        * @return A Clone (copy of the stored object)
        */
        [[nodiscard]] IntrinsicBase *Clone() const override;

    };

}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_veta::IntrinsicSpherical, "spherical")

namespace cereal {
    // This struct specialization will tell cereal which is the right way to serialize the ambiguity
    template<class Archive>
    struct specialize<Archive, ns_veta::IntrinsicSpherical, cereal::specialization::member_load_save> {
    };
}

CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::IntrinsicBase, ns_veta::IntrinsicSpherical)


#endif //VETA_SPHERICAL_H
