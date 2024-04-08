//
// Created by csl on 11/21/22.
//

#ifndef VETA_TYPE_DEF_HPP
#define VETA_TYPE_DEF_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>
#include <eigen3/Eigen/StdVector>

#include <initializer_list>
#include <memory>
#include <vector>
#include "set"
#include "map"

#include "veta/utils.hpp"

// Extend EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION with initializer list support.
#define EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(...)             \
  namespace std {                                                                \
    template <>                                                                  \
    class vector<__VA_ARGS__, std::allocator<__VA_ARGS__>>                       \
        : public vector<__VA_ARGS__, Eigen::aligned_allocator<__VA_ARGS__>> {    \
      typedef vector<__VA_ARGS__, Eigen::aligned_allocator<__VA_ARGS__>>         \
          vector_base;                                                           \
                                                                                 \
    public:                                                                      \
      typedef __VA_ARGS__ value_type;                                            \
      typedef vector_base::allocator_type allocator_type;                        \
      typedef vector_base::size_type size_type;                                  \
      typedef vector_base::iterator iterator;                                    \
      explicit vector(const allocator_type &a = allocator_type())                \
          : vector_base(a) {}                                                    \
      template <typename InputIterator>                                          \
      explicit vector(InputIterator first, InputIterator last,                   \
                      const allocator_type &a = allocator_type())                \
          : vector_base(first, last, a) {}                                       \
      vector(const vector &c) = default;                                         \
      explicit vector(size_type num, const value_type &val = value_type())       \
          : vector_base(num, val) {}                                             \
      explicit vector(iterator start, iterator end) : vector_base(start, end) {} \
      vector &operator=(const vector &x) = default;                              \
      /* Add initializer list constructor support*/                              \
      vector(std::initializer_list<__VA_ARGS__> list)                            \
          : vector_base(list.begin(), list.end()) {}                             \
    };                                                                           \
  } // namespace std

namespace ns_veta {
    // Portable type used to store an index
    using IndexT = uint64_t;

    // Portable value used to save an undefined index value
    static const IndexT UndefinedIndexT = std::numeric_limits<IndexT>::max();

    // the type of timestamp
    using TimeT = double;

    // undefined time stamp
    static const TimeT UndefinedTimeT = std::numeric_limits<TimeT>::min();

    // Standard Pair of IndexT
    using Pair = std::pair<IndexT, IndexT>;

    // Set of Pair
    using PairSet = std::set<Pair>;

    // Vector of Pair
    using PairVec = std::vector<Pair>;

    template<typename Key, typename Value>
    using HashMap = std::map<Key, Value, std::less<Key>,
            Eigen::aligned_allocator<std::pair<const Key, Value>>>;

    using Eigen::Map;

    // Trait used for double type
    using EigenDoubleTraits = Eigen::NumTraits<double>;

    template<typename ScaleType>
    using Vec2 = Eigen::Matrix<ScaleType, 2, 1>;

    template<typename ScaleType>
    using Vec3 = Eigen::Matrix<ScaleType, 3, 1>;

    template<typename ScaleType>
    using Vec4 = Eigen::Matrix<ScaleType, 4, 1>;

    template<typename ScaleType>
    using Vec6 = Eigen::Matrix<ScaleType, 6, 1>;

    template<typename ScaleType>
    using Vec9 = Eigen::Matrix<ScaleType, 9, 1>;

    template<typename ScaleType>
    using VecX = Eigen::Matrix<ScaleType, Eigen::Dynamic, 1>;

    // 3d vector using double internal format
    using Vec3d = Vec3<double>;

    // 2d vector using int internal format
    using Vec2i = Vec2<int>;

    // 2d vector using float internal format
    using Vec2f = Vec2<float>;

    // 3d vector using float internal format
    using Vec3f = Vec3<float>;

    // 9d vector using double internal format
    using Vec9d = Vec9<double>;

    // Quaternion type
    using Quaterniond = Eigen::Quaternion<double>;

    template<typename ScaleType>
    using Mat3 = Eigen::Matrix<ScaleType, 3, 3>;

    template<typename ScaleType>
    using RMat3 = Eigen::Matrix<ScaleType, 3, 3, Eigen::RowMajor>;

    template<typename ScaleType>
    using Mat34 = Eigen::Matrix<ScaleType, 3, 4>;

    template<typename ScaleType>
    using Mat4 = Eigen::Matrix<ScaleType, 4, 4>;

    template<typename ScaleType>
    using MatX = Eigen::Matrix<ScaleType, Eigen::Dynamic, Eigen::Dynamic>;

    // 3x3 matrix using double internal format
    using Mat3d = Mat3<double>;

    // 3x4 matrix using double internal format
    using Mat34d = Mat34<double>;

    // 2d vector using double internal format
    using Vec2d = Vec2<double>;

    // 4d vector using double internal format
    using Vec4d = Vec4<double>;

    // 6d vector using double internal format
    using Vec6d = Vec6<double>;

    // 4x4 matrix using double internal format
    using Mat4d = Mat4<double>;

    // generic matrix using unsigned int internal format
    using MatXui = MatX<unsigned int>;

    // 3x3 matrix using double internal format with RowMajor storage
    using RMat3d = RMat3<double>;

    // ---------------------------------
    // General purpose Matrix and Vector
    // ---------------------------------

    // Unconstrained matrix using double internal format
    using MatXd = MatX<double>;

    // Unconstrained vector using double internal format
    using VecXd = VecX<double>;

    // Unconstrained vector using unsigned int internal format
    using VecXui = VecX<unsigned int>;

    // Unconstrained matrix using float internal format
    using MatXf = MatX<float>;

    // Unconstrained vector using float internal format
    using VecXf = VecX<float>;

    template<typename ScaleType>
    using Mat2X = Eigen::Matrix<ScaleType, 2, Eigen::Dynamic>;

    template<typename ScaleType>
    using Mat3X = Eigen::Matrix<ScaleType, 3, Eigen::Dynamic>;

    template<typename ScaleType>
    using Mat4X = Eigen::Matrix<ScaleType, 4, Eigen::Dynamic>;

    template<typename ScaleType>
    using Mat9X = Eigen::Matrix<ScaleType, 9, Eigen::Dynamic>;

    // 2xN matrix using double internal format
    using Mat2Xd = Mat2X<double>;

    // 3xN matrix using double internal format
    using Mat3Xd = Mat3X<double>;

    // 4xN matrix using double internal format
    using Mat4Xd = Mat4X<double>;

    // Nx9 matrix using double internal format
    using MatX9d = Mat9X<double>;

    // -------------------------------------------
    // Sparse Matrix (Column major, and row major)
    // -------------------------------------------

    template<typename ScaleType>
    using SMat = Eigen::SparseMatrix<ScaleType>;

    template<typename ScaleType>
    using SRMat = Eigen::SparseMatrix<ScaleType, Eigen::RowMajor>;

    // Sparse unconstrained matrix using double internal format
    using SMatd = SMat<double>;

    /// Sparse unconstrained matrix using double internal format and Row Major storage
    using SRMatd = SRMat<double>;

} // namespace ns_veta

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Vec2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Vec3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Vec4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Vec6d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Vec9d)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Vec2i)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Vec2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Vec3f)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Quaterniond)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Mat3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::RMat3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Mat4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ns_veta::Mat34d)

#endif // VETA_TYPE_DEF_HPP
