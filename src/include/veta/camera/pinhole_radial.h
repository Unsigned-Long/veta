//
// Created by csl on 11/21/22.
//

#ifndef VETA_PINHOLE_RADIAL_H
#define VETA_PINHOLE_RADIAL_H

#include "veta/camera/pinhole.h"

namespace ns_veta {
    /**
    * @brief Solve by bisection the p' radius such that Square(disto(radius(p'))) = r^2
    * @param params Parameters of the distortion
    * @param r2 Target radius
    * @param functor Functor used to parametrize the distortion
    * @param epsilon Error driven threshold
    * @return Best radius
    */
    template<class Disto_Functor>
    double BisectionRadiusSolve(const std::vector<double> &params,
                                double r2, Disto_Functor &functor,
                                double epsilon = 1e-10) {
        // Guess plausible upper and lower bound
        double lowerBound = r2, upBound = r2;
        while (functor(params, lowerBound) > r2) {
            lowerBound /= 1.05;
        }
        while (functor(params, upBound) < r2) {
            upBound *= 1.05;
        }

        // Perform a bisection until epsilon accuracy is not reached
        while (epsilon < upBound - lowerBound) {
            const double mid = .5 * (lowerBound + upBound);
            if (functor(params, mid) > r2) {
                upBound = mid;
            } else {
                lowerBound = mid;
            }
        }
        return .5 * (lowerBound + upBound);
    }


    /**
     * @brief Implement a Pinhole camera with a 1 radial distortion coefficient.
     * \f$ x_d = x_u (1 + K_1 r^2 ) \f$
     */
    class PinholeIntrinsicRadialK1 : public PinholeIntrinsic {
        using class_type = PinholeIntrinsicRadialK1;

    protected:
        /// center of distortion is applied by the Intrinsics class
        std::vector<double> params_; // K1

    public:

        /**
        * @brief Constructor
        * @param w Width of the image
        * @param h Height of the image
        * @param focal Focal (in pixel) of the camera
        * @param ppx Principal point on X-Axis
        * @param ppy Principal point on Y-Axis
        * @param k1 Distortion coefficient
        */
        explicit PinholeIntrinsicRadialK1(int w = 0, int h = 0, double focal = 0.0,
                                          double ppx = 0, double ppy = 0, double k1 = 0.0)
                : PinholeIntrinsic(w, h, focal, ppx, ppy), params_({k1}) {}

        ~PinholeIntrinsicRadialK1() override = default;

        /**
        * @brief Tell from which type the embed camera is
        * @retval PINHOLE_CAMERA_RADIA_K1
        */
        [[nodiscard]] Eintrinsic GetType() const override;

        /**
        * @brief Does the camera model handle a distortion field?
        * @retval true if intrinsic holds distortion
        * @retval false if intrinsic does not hold distortion
        */
        [[nodiscard]] bool HaveDisto() const override;

        /**
        * @brief Add the distortion field to a point (that is in normalized camera frame)
        * @param p Point before distortion computation (in normalized camera frame)
        * @return point with distortion
        */
        [[nodiscard]] Vec2 AddDisto(const Vec2 &p) const override;

        /**
        * @brief Remove the distortion to a camera point (that is in normalized camera frame)
        * @param p Point with distortion
        * @return Point without distortion
        */
        [[nodiscard]] Vec2 RemoveDisto(const Vec2 &p) const override;

        /**
        * @brief Data wrapper for non linear optimization (get data)
        * @return vector of parameter of this intrinsic
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
        * @brief Return the un-distorted pixel (with removed distortion)
        * @param p Input distorted pixel
        * @return Point without distortion
        */
        [[nodiscard]] Vec2 GetUndistoPixel(const Vec2 &p) const override;

        /**
        * @brief Return the distorted pixel (with added distortion)
        * @param p Input pixel
        * @return Distorted pixel
        */
        [[nodiscard]] Vec2 GetDistoPixel(const Vec2 &p) const override;

        /**
        * @brief Serialization out
        * @param ar Archive
        */
        template<class Archive>
        inline void save(Archive &ar) const {
            PinholeIntrinsic::save(ar);
            ar(cereal::make_nvp("disto_k1", params_));
        }

        /**
        * @brief  Serialization in
        * @param ar Archive
        */
        template<class Archive>
        inline void load(Archive &ar) {
            PinholeIntrinsic::load(ar);
            ar(cereal::make_nvp("disto_k1", params_));
        }

        /**
        * @brief Clone the object
        * @return A Clone (copy of the stored object)
        */
        [[nodiscard]] IntrinsicBase *Clone() const override;

    private:

        /**
        * @brief Functor to solve Square(disto(radius(p'))) = r^2
        * @param params List of parameters (only the first one is used)
        * @param r2 square distance (relative to center)
        * @return distance
        */
        static inline double DistoFunctor(const std::vector<double> &params, double r2);
    };

    /**
    * @brief Implement a Pinhole camera with a 3 radial distortion coefficients.
    * \f$ x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) \f$
    */
    class PinholeIntrinsicRadialK3 : public PinholeIntrinsic {
        using class_type = PinholeIntrinsicRadialK3;

    protected:
        // center of distortion is applied by the Intrinsics class
        /// K1, K2, K3
        std::vector<double> params_;

    public:

        /**
        * @brief Constructor
        * @param w Width of image
        * @param h Height of image
        * @param focal Focal (in pixel) of the camera
        * @param ppx Principal point on X-Axis
        * @param ppy Principal point on Y-Axis
        * @param k1 First radial distortion coefficient
        * @param k2 Second radial distortion coefficient
        * @param k3 Third radial distortion coefficient
        */
        explicit PinholeIntrinsicRadialK3(int w = 0, int h = 0,
                                          double focal = 0.0, double ppx = 0, double ppy = 0,
                                          double k1 = 0.0, double k2 = 0.0, double k3 = 0.0)
                : PinholeIntrinsic(w, h, focal, ppx, ppy), params_({k1, k2, k3}) {}

        ~PinholeIntrinsicRadialK3() override = default;

        /**
        * @brief Tell from which type the embed camera is
        * @retval PINHOLE_CAMERA_RADIA_K3
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
        [[nodiscard]] Vec2 AddDisto(const Vec2 &p) const override;

        /**
        * @brief Remove the distortion to a camera point (that is in normalized camera frame)
        * @param p Point with distortion
        * @return Point without distortion
        */
        [[nodiscard]] Vec2 RemoveDisto(const Vec2 &p) const override;

        /**
        * @brief Data wrapper for non linear optimization (get data)
        * @return vector of parameter of this intrinsic
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
        * @brief Return the un-distorted pixel (with removed distortion)
        * @param p Input distorted pixel
        * @return Point without distortion
        */
        [[nodiscard]] Vec2 GetUndistoPixel(const Vec2 &p) const override;

        /**
        * @brief Return the distorted pixel (with added distortion)
        * @param p Input pixel
        * @return Distorted pixel
        */
        [[nodiscard]] Vec2 GetDistoPixel(const Vec2 &p) const override;

        /**
        * @brief Serialization out
        * @param ar Archive
        */
        template<class Archive>
        inline void save(Archive &ar) const {
            PinholeIntrinsic::save(ar);
            ar(cereal::make_nvp("disto_k3", params_));
        }

        /**
        * @brief  Serialization in
        * @param ar Archive
        */
        template<class Archive>
        inline void load(Archive &ar) {
            PinholeIntrinsic::load(ar);
            ar(cereal::make_nvp("disto_k3", params_));
        }

        /**
        * @brief Clone the object
        * @return A Clone (copy of the stored object)
        */
        [[nodiscard]] IntrinsicBase *Clone() const override;

    private:


        /**
        * @brief Functor to solve Square(disto(radius(p'))) = r^2
        * @param params List of the radial factors {k1, k2, k3}
        * @param r2 square distance (relative to center)
        * @return distance
        */
        static inline double DistoFunctor(const std::vector<double> &params, double r2);
    };

}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_veta::PinholeIntrinsicRadialK1, "pinhole_radial_k1")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::IntrinsicBase, ns_veta::PinholeIntrinsicRadialK1)
CEREAL_REGISTER_TYPE_WITH_NAME(ns_veta::PinholeIntrinsicRadialK3, "pinhole_radial_k3")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::IntrinsicBase, ns_veta::PinholeIntrinsicRadialK3)


#endif //VETA_PINHOLE_RADIAL_H
