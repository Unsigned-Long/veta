//
// Created by csl on 11/21/22.
//

#ifndef VETA_PINHOLE_BROWN_H
#define VETA_PINHOLE_BROWN_H

#include "veta/camera/pinhole.h"

namespace ns_veta {
    /**
    * @brief Implement a Pinhole camera with a 3 radial distortion coefficients and 2 tangential distortion coefficients.
    * \f$ x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) + (T_2 (r^2 + 2 x_u^2) + 2 T_1 x_u y_u) \f$
    * \f$ y_d = y_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) + (T_1 (r^2 + 2 y_u^2) + 2 T_2 x_u y_u) \f$
    */
    class Pinhole_Intrinsic_Brown_T2 : public Pinhole_Intrinsic {
        using class_type = Pinhole_Intrinsic_Brown_T2;

    protected:

        // center of distortion is applied by the Intrinsics class
        std::vector<double> params_; // K1, K2, K3, T1, T2

    public:

        /**
        * @brief Constructor
        * @param w Width of image
        * @param h Height of image
        * @param focal Focal distance (in pixel)
        * @param ppx Principal point on X-axis
        * @param ppy Principal point on Y-axis
        * @param k1 First radial distortion coefficient
        * @param k2 Second radial distortion coefficient
        * @param k3 Third radial distortion coefficient
        * @param t1 First tangential distortion coefficient
        * @param t2 Second tangential distortion coefficient
        */
        explicit Pinhole_Intrinsic_Brown_T2(int w = 0, int h = 0,
                                            double focal = 0.0, double ppx = 0, double ppy = 0,
                                            double k1 = 0.0, double k2 = 0.0, double k3 = 0.0,
                                            double t1 = 0.0, double t2 = 0.0);

        /**
        * @brief Get type of the intrinsic
        * @retval PINHOLE_CAMERA_BROWN
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
        * @note numerical approximation based on
        * Heikkila J (2000) Geometric Camera Calibration Using Circular Control Points.
        * IEEE Trans. Pattern Anal. Mach. Intell., 22:1066-1077
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
        subsetParameterization(const Intrinsic_Parameter_Type &parametrization) const override;

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
            Pinhole_Intrinsic::save(ar);
            ar(cereal::make_nvp("disto_t2", params_));
        }

        /**
        * @brief  Serialization in
        * @param ar Archive
        */
        template<class Archive>
        inline void load(Archive &ar) {
            Pinhole_Intrinsic::load(ar);
            ar(cereal::make_nvp("disto_t2", params_));
        }

        /**
        * @brief Clone the object
        * @return A clone (copy of the stored object)
        */
        [[nodiscard]] IntrinsicBase *clone() const override;

    private:


        /**
        * @brief Functor to calculate distortion offset accounting for both radial and tangential distortion
        * @param params List of parameters to define a Brown camera
        * @param p Input point
        * @return Transformed point
        */
        static Vec2 distoFunction(const std::vector<double> &params, const Vec2 &p);
    };

}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_veta::Pinhole_Intrinsic_Brown_T2, "pinhole_brown_t2")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::IntrinsicBase, ns_veta::Pinhole_Intrinsic_Brown_T2)


#endif //VETA_PINHOLE_BROWN_H