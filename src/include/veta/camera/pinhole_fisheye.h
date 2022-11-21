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
        using class_type = PinholeIntrinsicFisheye;

    protected:

        /// center of distortion is applied by the Intrinsics class
        std::vector<double> params_; // K1, K2, K3, K4


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
        explicit PinholeIntrinsicFisheye(int w = 0, int h = 0,
                                         double focal = 0.0, double ppx = 0, double ppy = 0,
                                         double k1 = 0.0, double k2 = 0.0, double k3 = 0.0, double k4 = 0.0);

        ~PinholeIntrinsicFisheye() override = default;

        /**
        * @brief Tell from which type the embed camera is
        * @retval PINHOLE_CAMERA_FISHEYE
        */
        [[nodiscard]] EINTRINSIC getType() const override;

        /**
        * @brief Does the camera model handle a distortion field?
        * @retval true
        */
        [[nodiscard]] bool have_disto() const override;

        /**
        * @brief Add the distortion field to a point (that is in normalized camera frame)
        * @param p Point before distortion computation (in normalized camera frame)
        * @return point with distortion
        */
        [[nodiscard]] Vec2 add_disto(const Vec2 &p) const override;

        /**
        * @brief Remove the distortion to a camera point (that is in normalized camera frame)
        * @param p Point with distortion
        * @return Point without distortion
        */
        [[nodiscard]] Vec2 remove_disto(const Vec2 &p) const override;

        /**
        * @brief Data wrapper for non linear optimization (get data)
        * @return vector of parameter of this intrinsic
        */
        [[nodiscard]] std::vector<double> getParams() const override;

        /**
        * @brief Data wrapper for non linear optimization (update from data)
        * @param params List of params used to update this intrinsic
        * @retval true if update is correct
        * @retval false if there was an error during update
        */
        bool updateFromParams(const std::vector<double> &params) override;

        /**
        * @brief Return the list of parameter indexes that must be held constant
        * @param parametrization The given parametrization
        */
        [[nodiscard]] std::vector<int>
        subsetParameterization(const IntrinsicParameterType &parametrization) const override;

        /**
        * @brief Return the un-distorted pixel (with removed distortion)
        * @param p Input distorted pixel
        * @return Point without distortion
        */
        [[nodiscard]] Vec2 get_ud_pixel(const Vec2 &p) const override;

        /**
        * @brief Return the distorted pixel (with added distortion)
        * @param p Input pixel
        * @return Distorted pixel
        */
        [[nodiscard]] Vec2 get_d_pixel(const Vec2 &p) const override;

        /**
        * @brief Serialization out
        * @param ar Archive
        */
        template<class Archive>
        inline void save(Archive &ar) const {
            PinholeIntrinsic::save(ar);
            ar(cereal::make_nvp("fisheye", params_));
        }

        /**
        * @brief  Serialization in
        * @param ar Archive
        */
        template<class Archive>
        inline void load(Archive &ar) {
            PinholeIntrinsic::load(ar);
            ar(cereal::make_nvp("fisheye", params_));
        }

        /**
        * @brief Clone the object
        * @return A clone (copy of the stored object)
        */
        [[nodiscard]] IntrinsicBase *clone() const override;
    };
}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_veta::PinholeIntrinsicFisheye, "fisheye")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::IntrinsicBase, ns_veta::PinholeIntrinsicFisheye)


#endif //VETA_PINHOLE_FISHEYE_H
