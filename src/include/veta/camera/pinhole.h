//
// Created by csl on 11/21/22.
//

#ifndef VETA_PINHOLE_H
#define VETA_PINHOLE_H

#include "veta/camera/intrinsics.h"

namespace ns_veta {
    /**
    * @brief Define an ideal Pinhole camera intrinsics (store a K 3x3 matrix)
    * with intrinsic parameters defining the K calibration matrix
    *
    * Intrinsic camera matrix is \f$ K = \begin{pmatrix} f & 0 & u_0 \\ 0 & f & v_0 \\ 0 & 0 & 1 \end{pmatrix} \f$
    *
    * @note This is an ideal Pinhole camera because it doesn't handle skew and distortion
    * @note The camera does only handle one focal length (ie: \f$ f_x = f_y = f \f$ )
    */
    class Pinhole_Intrinsic : public IntrinsicBase {
        using class_type = Pinhole_Intrinsic;

    protected:

        /// Intrinsic matrix : Focal & principal point are embed into the calibration matrix K
        Mat3 K_;

        /// Inverse of intrinsic matrix
        Mat3 Kinv_;

    public:

        /**
        * @brief Constructor
        * @param w Width of the image plane
        * @param h Height of the image plane
        * @param focal_length_pix Focal length (in pixel) of the camera
        * @param ppx Principal point on x-axis
        * @param ppy Principal point on y-axis
        */
        explicit Pinhole_Intrinsic(unsigned int w = 0, unsigned int h = 0, double focal_length_pix = 0.0,
                                   double ppx = 0.0, double ppy = 0.0);

        /**
        * @brief Constructor
        * @param w Width of the image plane
        * @param h Height of the image plane
        * @param K Intrinsic Matrix (3x3) {f,0,ppx; 0,f,ppy; 0,0,1}
        */
        Pinhole_Intrinsic(unsigned int w, unsigned int h, const Mat3 &K);

        /**
        * @brief Destructor
        */
        ~Pinhole_Intrinsic() override = default;

        /**
        * @brief Get type of the intrinsic
        * @retval PINHOLE_CAMERA
        */
        [[nodiscard]] EINTRINSIC getType() const override;

        /**
        * @brief Get the intrinsic matrix
        * @return 3x3 intrinsic matrix
        */
        [[nodiscard]] const Mat3 &K() const;

        /**
        * @brief Get the inverse of the intrinsic matrix
        * @return Inverse of intrinsic matrix
        */
        [[nodiscard]] const Mat3 &Kinv() const;


        /**
        * @brief Return the value of the focal in pixels
        * @return Focal of the camera (in pixel)
        */
        [[nodiscard]] inline double focal() const;

        /**
        * @brief Get principal point of the camera
        * @return Principal point of the camera
        */
        [[nodiscard]] inline Vec2 principal_point() const;

        /**
        * @brief Get bearing vectors from image coordinates
        * @return bearing vectors
        */
        Mat3X operator()(const Mat2X &points) const override;

        /**
        * @brief Transform a point from the camera plane to the image plane
        * @param p Camera plane point
        * @return Point on image plane
        */
        [[nodiscard]] Vec2 cam2ima(const Vec2 &p) const override;

        /**
        * @brief Transform a point from the image plane to the camera plane
        * @param p Image plane point
        * @return camera plane point
        */
        [[nodiscard]] Vec2 ima2cam(const Vec2 &p) const override;

        /**
        * @brief Does the camera model handle a distortion field?
        * @retval false if intrinsic does not hold distortion
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
        * @brief Normalize a given unit pixel error to the camera plane
        * @param value Error in image plane
        * @return error of passing from the image plane to the camera plane
        */
        [[nodiscard]] double imagePlane_toCameraPlaneError(double value) const override;

        /**
        * @brief Return the projection matrix (interior & exterior) as a simplified projective projection
        * @param pose Extrinsic matrix
        * @return Concatenation of intrinsic matrix and extrinsic matrix
        */
        [[nodiscard]] Mat34 get_projective_equivalent(const Pose &pose) const override;


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
            IntrinsicBase::save(ar);
            ar(cereal::make_nvp("focal_length", K_(0, 0)));
            const std::vector<double> pp{K_(0, 2), K_(1, 2)};
            ar(cereal::make_nvp("principal_point", pp));
        }


        /**
        * @brief  Serialization in
        * @param ar Archive
        */
        template<class Archive>
        inline void load(Archive &ar) {
            IntrinsicBase::load(ar);
            double focal_length;
            ar(cereal::make_nvp("focal_length", focal_length));
            std::vector<double> pp(2);
            ar(cereal::make_nvp("principal_point", pp));
            *this = Pinhole_Intrinsic(w_, h_, focal_length, pp[0], pp[1]);
        }

        /**
        * @brief Clone the object
        * @return A clone (copy of the stored object)
        */
        [[nodiscard]] IntrinsicBase *clone() const override;
    };

}


CEREAL_REGISTER_TYPE_WITH_NAME(ns_veta::Pinhole_Intrinsic, "pinhole")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::IntrinsicBase, ns_veta::Pinhole_Intrinsic)


#endif //VETA_PINHOLE_H
