//
// Created by csl on 11/21/22.
//

#ifndef VETA_UTILS_HPP
#define VETA_UTILS_HPP

#include <functional>

#include "cereal/cereal.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
#include <cereal/archives/binary.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>

namespace ns_veta {
    template<class T>
    inline void hash_combine(std::size_t &seed, const T &v) {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    /**
    * @brief Compute square of a number
    * @tparam T Type of the number to square
    * @param x Input number
    * @return square of x
    */
    template<typename T>
    inline T Square(T x) {
        return x * x;
    }

#define SUM_OR_DYNAMIC(x, y) (x==Eigen::Dynamic||y==Eigen::Dynamic)?Eigen::Dynamic:(x+y)

    template<typename Derived1, typename Derived2>
    struct hstack_return {
        using Scalar = typename Derived1::Scalar;
        enum {
            RowsAtCompileTime = Derived1::RowsAtCompileTime,
            ColsAtCompileTime = SUM_OR_DYNAMIC(Derived1::ColsAtCompileTime, Derived2::ColsAtCompileTime),
            Options = Derived1::Flags & Eigen::RowMajorBit ? Eigen::RowMajor : 0,
            MaxRowsAtCompileTime = Derived1::MaxRowsAtCompileTime,
            MaxColsAtCompileTime = SUM_OR_DYNAMIC(Derived1::MaxColsAtCompileTime, Derived2::MaxColsAtCompileTime)
        };
        using type = Eigen::Matrix<
                Scalar,
                RowsAtCompileTime, ColsAtCompileTime,
                Options,
                MaxRowsAtCompileTime, MaxColsAtCompileTime
        >;
    };

    template<typename Derived1, typename Derived2>
    typename hstack_return<Derived1, Derived2>::type
    HStack(const Eigen::MatrixBase<Derived1> &lhs, const Eigen::MatrixBase<Derived2> &rhs) {
        typename hstack_return<Derived1, Derived2>::type res;
        res.resize(lhs.rows(), lhs.cols() + rhs.cols());
        res << lhs, rhs;
        return res;
    }

#ifdef MSWINDOWS
    static const char* separator_set = "\\/";
  static const char preferred_separator = '\\';
#else
    static const char *separator_set = "/";
    static const char preferred_separator = '/';
#endif

    inline bool is_separator(char ch) {
        for (int i = 0; separator_set[i]; i++) {
            if (separator_set[i] == ch)
                return true;
        }
        return false;
    }

    inline std::string filename_part(const std::string &spec) {
        // scan back through filename until a preferred_separator is found and remove prefix;
        // if there is no preferred_separator then remove nothing, i.e. the whole filespec is filename
        size_t i = spec.size();
        while (i--) {
            if (is_separator(spec[i]))
                return spec.substr(i + 1, spec.size() - i - 1);
        }
        return spec;
    }

    inline std::string extension_part(const std::string &spec) {
        std::string fname = filename_part(spec);
        // scan back through filename until a '.' is found and remove prefix;
        std::string::size_type i = fname.find_last_of('.');
        // observe Unix convention that a dot at the start of a filename is part of the name, not the extension;
        if (i != 0 && i != std::string::npos)
            fname.erase(0, i + 1);
        else
            fname.erase();
        return fname;
    }

    /// Allow to select the Keys of a map.
    struct RetrieveKey {
        template<typename T>
        typename T::first_type operator()(const T &keyValuePair) const {
            return keyValuePair.first;
        }
    };

}

#endif //VETA_UTILS_HPP
