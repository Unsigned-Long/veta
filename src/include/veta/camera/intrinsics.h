//
// Created by csl on 11/21/22.
//

#ifndef VETA_INTRINSICS_H
#define VETA_INTRINSICS_H

#include "veta/type_def.hpp"
#include "veta/pose.h"

namespace ns_veta {
    /**
    * @brief Struct used to force "clonability"
    */
    template<typename T>
    struct Clonable {
        [[nodiscard]] virtual T *Clone() const = 0;

        virtual ~Clonable() = default;
    };

    /**
    * @enum Eintrinsic List of usable camera Intrinsics
    * @var PINHOLE_CAMERA
    *   Pinhole camera is an ideal pinhole camera with 3x3 intrinsics matrix : \n
    *      \f$ K=\begin{pmatrix} f & 0 & u_0 \\ 0 & f & v_0 \\ 0 & 0 & 1 \end{pmatrix} \f$
    * @var PINHOLE_CAMERA_RADIA_K1
    *   Same as PINHOLE_CAMERA but before projection, pixel are distorted using radial distortion using one parameter \f$k_1\f$ \n
    *    Assuming input pixel is \f$X\f$, distorted pixel \f$X_d\f$ is given by the relation : \n
    *      \f$ X_d = ( 1 + k_1 r^2) X \f$  \n
    *    Where \f$ r^2 = (X_x - u_0)^2 + (Y_y - v_0)^2 \f$
    * @var PINHOLE_CAMERA_RADIA_K3
    *   Same as PINHOLE_CAMERA_RADIA_K1 but using 3 parameters \f$k_1, k_2, k_3\f$ : \n
    *      \f$ X_d = ( 1 + k_1 r^2 + k_2 r^4 + k_3 r^6 ) X \f$
    * @var PINHOLE_CAMERA_BROWN_T2
    *   Same as PINHOLE_CAMERA with radial distortion and Tangential distortion : \n
    *      \f$ x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) + (T_2 (r^2 + 2 x_u^2) + 2 T_1 x_u y_u) \f$
    *      \f$ y_d = y_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) + (T_1 (r^2 + 2 y_u^2) + 2 T_2 x_u y_u) \f$
    * @var PINHOLE_CAMERA_FISHEYE
    *   Simple fisheye camera with 4 distortion coefficients
    */
    enum Eintrinsic {
        PINHOLE_CAMERA_START = 0,
        PINHOLE_CAMERA,           // No distortion
        PINHOLE_CAMERA_RADIA_K1,  // radial distortion K1
        PINHOLE_CAMERA_RADIA_K3,  // radial distortion K1,K2,K3
        PINHOLE_CAMERA_BROWN_T2,  // radial distortion K1,K2,K3, tangential distortion T1,T2
        PINHOLE_CAMERA_FISHEYE,   // a simple Fish-eye distortion model with 4 distortion coefficients
        PINHOLE_CAMERA_END,
        CAMERA_SPHERICAL = PINHOLE_CAMERA_END + 1
    };

    /**
    * @brief test if given intrinsic value corresponds to a pinhole
    * @param eintrinsic Intrinsic value to test
    * @retval true if parameter is a pinhole
    * @retval false if parameter is not a pinhole
    */
    static inline bool IsPinhole(Eintrinsic eintrinsic) {
        return eintrinsic > PINHOLE_CAMERA_START && eintrinsic < PINHOLE_CAMERA_END;
    }

    static inline bool IsSpherical(Eintrinsic eintrinsic) {
        return eintrinsic == CAMERA_SPHERICAL;
    }

    /**
    * @brief Test if given intrinsic value is valid
    * @param eintrinsic Intrinsic value to test
    * @retval true if parameter is valid
    * @retval false if parameter is invalid
    */
    static inline bool IsValid(Eintrinsic eintrinsic) {
        return IsPinhole(eintrinsic) || IsSpherical(eintrinsic);
    }

    /**
    * @enum IntrinsicParamType Used to control which camera parameter must be \n
       considered as variable of held constant for non linear refinement
    * @var NONE
    *   Intrinsic parameters will be considered as FIXED
    * @var ADJUST_FOCAL_LENGTH
    *   Focal length will be considered as variable for refinement
    * @var ADJUST_PRINCIPAL_POINT
    *   Principal point will be considered as variable for refinement
    * @var ADJUST_DISTORTION
    *   Distortion parameters will be considered as variable for refinement
    * @var ADJUST_ALL
    *   All parameters will be considered as variable for refinement
    */
    enum class IntrinsicParamType : int {
        // Note: Use power of two values in order to use bitwise operators.
        NONE = 1, // All parameters will be held constant
        ADJUST_FOCAL_LENGTH = 2,
        ADJUST_PRINCIPAL_POINT = 4,
        ADJUST_DISTORTION = 8,
        ADJUST_ALL = ADJUST_FOCAL_LENGTH | ADJUST_PRINCIPAL_POINT | ADJUST_DISTORTION
    };

    inline constexpr IntrinsicParamType
    operator|(IntrinsicParamType x, IntrinsicParamType y) {
        return static_cast<IntrinsicParamType>
        (static_cast<std::underlying_type<IntrinsicParamType>::type>(x) |
         static_cast<std::underlying_type<IntrinsicParamType>::type>(y));
    }

    inline constexpr IntrinsicParamType
    operator&(IntrinsicParamType x, IntrinsicParamType y) {
        return static_cast<IntrinsicParamType>
        (static_cast<std::underlying_type<IntrinsicParamType>::type>(x) &
         static_cast<std::underlying_type<IntrinsicParamType>::type>(y));
    }


    /**
    * @brief Base class used to store common intrinsics parameters
    */
    struct IntrinsicBase : public Clonable<IntrinsicBase> {
        // Width of image
        unsigned int w_;
        // Height of image
        unsigned int h_;

        /**
        * @brief Constructor
        * @param w Width of the image
        * @param h Height of the image
        */
        explicit IntrinsicBase(unsigned int w = 0, unsigned int h = 0) : w_(w), h_(h) {}

        /**
        * @brief Destructor
        */
        ~IntrinsicBase() override = default;

        /**
        * @brief Get width of the image
        * @return width of the image
        */
        [[nodiscard]] unsigned int w() const;

        /**
        * @brief Get height of the image
        * @return height of the image
        */
        [[nodiscard]] unsigned int h() const;

        /**
        * @brief Compute projection of a 3D point into the image plane
        * (Apply disto (if any) and Intrinsics)
        * @param X 3D-point to Project on image plane
        * @return Projected (2D) point on image plane
        */
        [[nodiscard]] virtual Vec2 Project(const Vec3 &X, bool ignore_distortion) const;

        /**
        * @brief Compute the Residual between the 3D projected point and an image observation
        * @param X 3d point to Project on camera plane
        * @param x image observation
        * @brief Relative 2d distance between projected and observed points
        */
        [[nodiscard]] Vec2 Residual(const Vec3 &X, const Vec2 &x, bool ignore_distortion = false) const;

        // ---------------
        // Virtual members
        // ---------------

        /**
        * @brief Tell from which type the embed camera is
        * @return Corresponding intrinsic
        */
        [[nodiscard]] virtual Eintrinsic GetType() const = 0;

        /**
        * @brief Data wrapper for non linear optimization (get data)
        * @return vector of parameter of this intrinsic
        */
        [[nodiscard]] virtual std::vector<double> GetParams() const = 0;

        /**
        * @brief Data wrapper for non linear optimization (update from data)
        * @param params List of params used to update this intrinsic
        * @retval true if update is correct
        * @retval false if there was an error during update
        */
        virtual bool UpdateFromParams(const std::vector<double> &params) = 0;

        /**
        * @brief Return the list of parameter indexes that must be held constant
        * @param parametrization The given parametrization
        */
        [[nodiscard]] virtual std::vector<int>
        SubsetParameterization(const IntrinsicParamType &parametrization) const = 0;

        /**
        * @brief Get bearing vectors from image coordinates
        * @return bearing vectors
        */
        virtual Mat3X operator()(const Mat2X &p) const = 0;

        /**
        * @brief Transform a point from the camera plane to the image plane
        * @param p Camera plane point
        * @return Point on image plane
        */
        [[nodiscard]] virtual Vec2 CamToImg(const Vec2 &p) const = 0;

        /**
        * @brief Transform a point from the image plane to the camera plane
        * @param p Image plane point
        * @return camera plane point
        */
        [[nodiscard]] virtual Vec2 ImgToCam(const Vec2 &p) const = 0;

        /**
        * @brief Does the camera model handle a distortion field?
        * @retval true if intrinsic holds distortion
        * @retval false if intrinsic does not hold distortion
        */
        [[nodiscard]] virtual bool HaveDisto() const;

        /**
        * @brief Add the distortion field to a point (that is in normalized camera frame)
        * @param p Point before distortion computation (in normalized camera frame)
        * @return point with distortion
        */
        [[nodiscard]] virtual Vec2 AddDisto(const Vec2 &p) const = 0;

        /**
        * @brief Remove the distortion to a camera point (that is in normalized camera frame)
        * @param p Point with distortion
        * @return Point without distortion
        */
        [[nodiscard]] virtual Vec2 RemoveDisto(const Vec2 &p) const = 0;

        /**
        * @brief Return the un-distorted pixel (with removed distortion)
        * @param p Input distorted pixel
        * @return Point without distortion
        */
        [[nodiscard]] virtual Vec2 GetUndistoPixel(const Vec2 &p) const = 0;

        /**
        * @brief Return the distorted pixel (with added distortion)
        * @param p Input pixel
        * @return Distorted pixel
        */
        [[nodiscard]] virtual Vec2 GetDistoPixel(const Vec2 &p) const = 0;

        /**
        * @brief Normalize a given unit pixel error to the camera plane
        * @param value Error in image plane
        * @return error of passing from the image plane to the camera plane
        */
        [[nodiscard]] virtual double ImagePlaneToCameraPlaneError(double value) const = 0;

        /**
        * @brief Return the projection matrix (interior & exterior) as a simplified projective projection
        * @param pose Extrinsic matrix
        * @return Concatenation of intrinsic matrix and extrinsic matrix
        */
        [[nodiscard]] virtual Mat34 GetProjectiveEquivalent(const Pose &pose) const = 0;

        /**
        * @brief Serialization out
        * @param ar Archive
        */
        template<class Archive>
        void save(Archive &ar) const {
            ar(cereal::make_nvp("width", w_));
            ar(cereal::make_nvp("height", h_));
        }

        /**
        * @brief  Serialization in
        * @param ar Archive
        */
        template<class Archive>
        void load(Archive &ar) {
            ar(cereal::make_nvp("width", w_));
            ar(cereal::make_nvp("height", h_));
        }

        /**
        * @brief Generate a unique Hash from the camera parameters (used for grouping)
        * @return Hash value
        */
        [[nodiscard]] virtual std::size_t HashValue() const;
    };

}


#endif //VETA_INTRINSICS_H
