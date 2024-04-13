//
// Created by csl on 11/21/22.
//

#ifndef VETA_PINHOLE_H
#define VETA_PINHOLE_H

#include "veta/camera/intrinsics.h"

namespace ns_veta {
    /**
    * @brief Define an ideal Pinhole camera intrinsics (store a KMat 3x3 matrix)
    * with intrinsic parameters defining the KMat calibration matrix
    *
    * Intrinsic camera matrix is \f$ KMat = \begin{pmatrix} f & 0 & u_0 \\ 0 & f & v_0 \\ 0 & 0 & 1 \end{pmatrix} \f$
    *
    * @note This is an ideal Pinhole camera because it doesn't handle skew and distortion
    * @note The camera does only handle one Focal length (ie: \f$ f_x = f_y = f \f$ )
    */
    class PinholeIntrinsic : public IntrinsicBase {
    public:
        using Ptr = std::shared_ptr<PinholeIntrinsic>;

        using class_type = PinholeIntrinsic;

    protected:

        // Intrinsic matrix : Focal & principal point are embed into the calibration matrix KMat
        Mat3d K;

        // Inverse of intrinsic matrix
        Mat3d KInv;

    public:

        /**
        * @brief Constructor
        * @param w Width of the image plane
        * @param h Height of the image plane
        * @param focalLengthPix Focal length (in pixel) of the camera
        * @param ppx Principal point on x-axis
        * @param ppy Principal point on y-axis
        */
        explicit PinholeIntrinsic(unsigned int w, unsigned int h, double fx, double fy, double ppx, double ppy);

        PinholeIntrinsic() = default;

        /**
        * @brief Constructor
        * @param w Width of the image plane
        * @param h Height of the image plane
        * @param K Intrinsic Matrix (3x3) {f,0,ppx; 0,f,ppy; 0,0,1}
        */
        PinholeIntrinsic(unsigned int w, unsigned int h, Mat3d KMat);

        static Ptr Create(unsigned int w, unsigned int h, double fx, double fy, double ppx, double ppy);

        static Ptr Create(unsigned int w, unsigned int h, const Mat3d &KMat);

        /**
        * @brief Destructor
        */
        ~PinholeIntrinsic() override = default;

        /**
        * @brief Get type of the intrinsic
        * @retval PINHOLE_CAMERA
        */
        [[nodiscard]] Eintrinsic GetType() const override;

        /**
        * @brief Get the intrinsic matrix
        * @return 3x3 intrinsic matrix
        */
        [[nodiscard]] const Mat3d &KMat() const;

        double *FXAddress();

        double *FYAddress();

        double *CXAddress();

        double *CYAddress();

        /**
        * @brief Get the Inverse of the intrinsic matrix
        * @return Inverse of intrinsic matrix
        */
        [[nodiscard]] const Mat3d &KInvMat() const;


        /**
        * @brief Return the value of the Focal in pixels
        * @return Focal of the camera (in pixel)
        */
        [[nodiscard]] double Focal() const;

        [[nodiscard]] double FocalX() const;

        [[nodiscard]] double FocalY() const;

        [[nodiscard]] Vec2d FocalXY() const;

        /**
        * @brief Get principal point of the camera
        * @return Principal point of the camera
        */
        [[nodiscard]] Vec2d PrincipalPoint() const;

        /**
        * @brief Get bearing vectors from image coordinates
        * @return bearing vectors
        */
        Mat3Xd operator()(const Mat2Xd &points) const override;

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
        * @brief Does the camera model handle a distortion field?
        * @retval false if intrinsic does not hold distortion
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
            IntrinsicBase::save(ar);
            const std::vector<double> focalLength{K(0, 0), K(1, 1)};
            ar(cereal::make_nvp("focal_length", focalLength));
            const std::vector<double> pp{K(0, 2), K(1, 2)};
            ar(cereal::make_nvp("principal_point", pp));
        }


        /**
        * @brief  Serialization in
        * @param ar Archive
        */
        template<class Archive>
        inline void load(Archive &ar) {
            IntrinsicBase::load(ar);
            std::vector<double> focalLength(2);
            ar(cereal::make_nvp("focal_length", focalLength));
            std::vector<double> pp(2);
            ar(cereal::make_nvp("principal_point", pp));
            *this = PinholeIntrinsic(imgWidth, imgHeight, focalLength[0], focalLength[1], pp[0], pp[1]);
        }

        /**
        * @brief Clone the object
        * @return A Clone (copy of the stored object)
        */
        [[nodiscard]] IntrinsicBase *Clone() const override;
    };

}

CEREAL_REGISTER_TYPE_WITH_NAME(ns_veta::PinholeIntrinsic, "pinhole")
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::IntrinsicBase, ns_veta::PinholeIntrinsic)


#endif //VETA_PINHOLE_H
