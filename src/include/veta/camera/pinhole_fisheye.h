//
// Created by csl on 11/21/22.
//

#ifndef VETA_PINHOLE_FISHEYE_H
#define VETA_PINHOLE_FISHEYE_H

#include "pinhole.h"

namespace ns_veta {
    /**
    * @brief Implement a simple Fish-eye camera model
    */
    class PinholeIntrinsicFisheye : public PinholeIntrinsic {
    public:
        using Ptr = std::shared_ptr<PinholeIntrinsicFisheye>;

        using class_type = PinholeIntrinsicFisheye;

    protected:

        /// Center of distortion is applied by the Intrinsics class
        std::vector<double> params; // K1, K2, K3, K4


    public:

        /**
        * @brief Constructor
        * @param w Width of image plane
        * @param h Height of image plane
        * @param focal Focal distance in pixel
        * @param ppx Principal point on X-axis
        * @param ppy Principal point on Y-axis
        * @param k1 Distortion coefficient
        * @param k2 Distortion coefficient
        * @param k3 Distortion coefficient
        * @param k4 Distortion coefficient
        */
        explicit PinholeIntrinsicFisheye(int w, int h, double fx, double fy, double ppx, double ppy,
                                         double k1 = 0.0, double k2 = 0.0, double k3 = 0.0, double k4 = 0.0);

        PinholeIntrinsicFisheye() = default;

        ~PinholeIntrinsicFisheye() override = default;

        static Ptr Create(int w, int h, double fx, double fy, double ppx, double ppy,
                          double k1 = 0.0, double k2 = 0.0, double k3 = 0.0, double k4 = 0.0);

        /**
        * @brief Tell from which type the embed camera is
        * @retval PINHOLE_CAMERA_FISHEYE
        */
        [[nodiscard]] Eintrinsic GetType() const override;

        /**
        * @brief Does the camera model handle a distortion field?
        * @retval true
        */
        [[nodiscard]] bool HaveDisto() const override;

        /**
        * @brief Add the distortion field to a point (that is in normalized camera frame)
        * @param p Point before distortion computation (in normalized camera frame)
        * @return point with distortion
        */
        [[nodiscard]] Vec2d AddDisto(const Vec2d &p) const override;

        /**
        * @brief Remove the distortion to a camera point (that is in normalized camera frame)
        * @param p Point with distortion
        * @return Point without distortion
        */
        [[nodiscard]] Vec2d RemoveDisto(const Vec2d &p) const override;

        /**
        * @brief Data wrapper for non linear optimization (get data)
        * @return vector of parameter of this intrinsic
        */
        [[nodiscard]] std::vector<double> GetParams() const override;

        /**
        * @brief Data wrapper for non linear optimization (update from data)
        * @param paramsVec List of paramsVec used to update this intrinsic
        * @retval true if update is correct
        * @retval false if there was an error during update
        */
        bool UpdateFromParams(const std::vector<double> &paramsVec) override;

        /**
        * @brief Return the list of parameter indexes that must be held constant
        * @param parametrization The given parametrization
        */
        [[nodiscard]] std::vector<int>
        SubsetParameterization(const IntrinsicParamType &parametrization) const override;

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
        * @brief Serialization out
        * @param ar Archive
        */
        template<class Archive>
        inline void save(Archive &ar) const {
            PinholeIntrinsic::save(ar);
            ar(cereal::make_nvp("disto_param", params));
        }

        /**
        * @brief  Serialization in
        * @param ar Archive
        */
        template<class Archive>
        inline void load(Archive &ar) {
            PinholeIntrinsic::load(ar);
            ar(cereal::make_nvp("disto_param", params));
            if (params.size() != 4) {
                throw std::runtime_error(
                        "camera model 'pinhole_fisheye' should maintain four distortion parameters (k1, k2, k3, k4)"
                );
            }
        }

        /**
        * @brief Clone the object
        * @return A Clone (copy of the stored object)
        */
        [[nodiscard]] IntrinsicBase *Clone() const override;
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_veta::PinholeIntrinsicFisheye, "pinhole_fisheye")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::IntrinsicBase, ns_veta::PinholeIntrinsicFisheye)
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::PinholeIntrinsic, ns_veta::PinholeIntrinsicFisheye)


#endif //VETA_PINHOLE_FISHEYE_H
