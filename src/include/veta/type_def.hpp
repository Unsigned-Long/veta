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
    using IndexT = uint32_t;

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

    // 3d vector using double internal format
    using Vec3d = Eigen::Vector3d;

    // 2d vector using int internal format
    using Vec2i = Eigen::Vector2i;

    // 2d vector using float internal format
    using Vec2f = Eigen::Vector2f;

    // 3d vector using float internal format
    using Vec3f = Eigen::Vector3f;

    // 9d vector using double internal format
    using Vec9d = Eigen::Matrix<double, 9, 1>;

    // Quaternion type
    using Quaterniond = Eigen::Quaternion<double>;

    // 3x3 matrix using double internal format
    using Mat3d = Eigen::Matrix<double, 3, 3>;

    // 3x4 matrix using double internal format
    using Mat34d = Eigen::Matrix<double, 3, 4>;

    // 2d vector using double internal format
    using Vec2d = Eigen::Vector2d;

    // 4d vector using double internal format
    using Vec4d = Eigen::Vector4d;

    // 6d vector using double internal format
    using Vec6d = Eigen::Matrix<double, 6, 1>;

    // 4x4 matrix using double internal format
    using Mat4d = Eigen::Matrix<double, 4, 4>;

    // generic matrix using unsigned int internal format
    using MatXui = Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>;

    // 3x3 matrix using double internal format with RowMajor storage
    using RMat3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

    // ---------------------------------
    // General purpose Matrix and Vector
    // ---------------------------------

    // Unconstrained matrix using double internal format
    using MatXd = Eigen::MatrixXd;

    // Unconstrained vector using double internal format
    using VecXd = Eigen::VectorXd;

    // Unconstrained vector using unsigned int internal format
    using VecXui = Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>;

    // Unconstrained matrix using float internal format
    using MatXf = Eigen::MatrixXf;

    // Unconstrained vector using float internal format
    using VecXf = Eigen::VectorXf;

    // 2xN matrix using double internal format
    using Mat2Xd = Eigen::Matrix<double, 2, Eigen::Dynamic>;

    // 3xN matrix using double internal format
    using Mat3Xd = Eigen::Matrix<double, 3, Eigen::Dynamic>;

    // 4xN matrix using double internal format
    using Mat4Xd = Eigen::Matrix<double, 4, Eigen::Dynamic>;

    // Nx9 matrix using double internal format
    using MatX9d = Eigen::Matrix<double, Eigen::Dynamic, 9>;

    // -------------------------------------------
    // Sparse Matrix (Column major, and row major)
    // -------------------------------------------

    // Sparse unconstrained matrix using double internal format
    using SMat = Eigen::SparseMatrix<double>;

    /// Sparse unconstrained matrix using double internal format and Row Major storage
    using SRMat = Eigen::SparseMatrix<double, Eigen::RowMajor>;

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
