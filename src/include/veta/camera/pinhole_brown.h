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
    class PinholeIntrinsicBrownT2 : public PinholeIntrinsic {
    public:
        using Ptr = std::shared_ptr<PinholeIntrinsicBrownT2>;

        using class_type = PinholeIntrinsicBrownT2;

    protected:

        // Center of distortion is applied by the Intrinsics class
        std::vector<double> params; // K1, K2, K3, T1, T2

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
        explicit PinholeIntrinsicBrownT2(int w, int h, double fx, double fy, double ppx, double ppy,
                                         double k1 = 0.0, double k2 = 0.0, double k3 = 0.0,
                                         double t1 = 0.0, double t2 = 0.0);

        PinholeIntrinsicBrownT2() = default;

        static Ptr Create(int w, int h, double fx, double fy, double ppx, double ppy,
                          double k1 = 0.0, double k2 = 0.0, double k3 = 0.0,
                          double t1 = 0.0, double t2 = 0.0);

        /**
        * @brief Get type of the intrinsic
        * @retval PINHOLE_CAMERA_BROWN_T2
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
        * @note numerical approximation based on
        * Heikkila J (2000) Geometric Camera Calibration Using Circular Control Points.
        * IEEE Trans. Pattern Anal. Mach. Intell., 22:1066-1077
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
            if (params.size() != 5) {
                throw std::runtime_error(
                        "camera model 'pinhole_brown_t2' should maintain five distortion parameters (k1, k2, k3, t1, t2)"
                );
            }
        }

        /**
        * @brief Clone the object
        * @return A Clone (copy of the stored object)
        */
        [[nodiscard]] IntrinsicBase *Clone() const override;

    private:


        /**
        * @brief Functor to calculate distortion offset accounting for both radial and tangential distortion
        * @param params List of parameters to define a Brown camera
        * @param p Input point
        * @return Transformed point
        */
        static Vec2d DistoFunction(const std::vector<double> &params, const Vec2d &p);
    };

}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_veta::PinholeIntrinsicBrownT2, "pinhole_brown_t2")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::IntrinsicBase, ns_veta::PinholeIntrinsicBrownT2)
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::PinholeIntrinsic, ns_veta::PinholeIntrinsicBrownT2)

#endif //VETA_PINHOLE_BROWN_H
