//
// Created by csl on 11/21/22.
//
#include "iostream"
#include "veta/veta.h"
#include "veta/camera/pinhole_radial.h"
#include "veta/camera/pinhole_brown.h"
#include "veta/camera/pinhole_fisheye.h"
#include "veta/camera/spherical.h"

int main(int argc, char **argv) {
    ns_veta::Veta veta;

    {
        // view
        {
            auto view = ns_veta::View::Create(ns_veta::UndefinedTimeT, 0, 0, 0, 200, 100);
            veta.views.insert(std::make_pair(view->viewId, view));
        }
        {
            auto view = ns_veta::View::Create(ns_veta::UndefinedTimeT, 1, 1, 0, 200, 100);
            veta.views.insert(std::make_pair(view->viewId, view));
        }
        {
            auto view = ns_veta::View::Create(ns_veta::UndefinedTimeT, 2, 2, 0, 200, 100);
            veta.views.insert(std::make_pair(view->viewId, view));
        }
        {
            auto view = ns_veta::View::Create(ns_veta::UndefinedTimeT, 3, 3, 0, 200, 100);
            veta.views.insert(std::make_pair(view->viewId, view));
        }
        {
            auto view = ns_veta::View::Create(ns_veta::UndefinedTimeT, 4, 4, 0, 200, 100);
            veta.views.insert(std::make_pair(view->viewId, view));
        }
    }
    {
        // pose
        auto pose = ns_veta::Posed(ns_veta::Mat3d::Identity(), ns_veta::Vec3d::Zero());
        veta.poses.insert(std::make_pair(0, pose));
    }
    {
        // intri
        auto pinholeIntrinsicBrownT2 = ns_veta::PinholeIntrinsicBrownT2::Create(200, 100, 160, 140, 100, 50);
        veta.intrinsics.insert(std::make_pair(0, pinholeIntrinsicBrownT2));
        auto pinholeIntrinsicRadialK1 = ns_veta::PinholeIntrinsicRadialK1::Create(200, 100, 160, 140, 100, 50);
        veta.intrinsics.insert(std::make_pair(1, pinholeIntrinsicRadialK1));
        auto pinholeIntrinsicRadialK3 = ns_veta::PinholeIntrinsicRadialK3::Create(200, 100, 160, 140, 100, 50);
        veta.intrinsics.insert(std::make_pair(2, pinholeIntrinsicRadialK3));
        auto pinholeIntrinsicFisheye = ns_veta::PinholeIntrinsicFisheye::Create(200, 100, 160, 140, 100, 50);
        veta.intrinsics.insert(std::make_pair(3, pinholeIntrinsicFisheye));
        auto spherical = ns_veta::IntrinsicSpherical::Create(200, 100);
        veta.intrinsics.insert(std::make_pair(4, spherical));
    }
    {
        // structure
        auto landmark = ns_veta::Landmark(
                ns_veta::Vec3d::Zero(), {{0, ns_veta::Observation(ns_veta::Vec2d::Zero(), 0)}}
        );
        veta.structure.insert(std::make_pair(0, landmark));
    }

    ns_veta::Save(veta, "/home/csl/CppWorks/artwork/veta/output/veta.json", ns_veta::Veta::ALL);
    ns_veta::Save(veta, "/home/csl/CppWorks/artwork/veta/output/veta.bin", ns_veta::Veta::ALL);
    ns_veta::Save(veta, "/home/csl/CppWorks/artwork/veta/output/veta.xml", ns_veta::Veta::ALL);

    ns_veta::Load(veta, "/home/csl/CppWorks/artwork/veta/output/veta.json", ns_veta::Veta::ALL);
    ns_veta::Load(veta, "/home/csl/CppWorks/artwork/veta/output/veta.bin", ns_veta::Veta::ALL);
    ns_veta::Load(veta, "/home/csl/CppWorks/artwork/veta/output/veta.xml", ns_veta::Veta::ALL);

    ns_veta::Save(veta, "/home/csl/CppWorks/artwork/veta/output/veta.json", ns_veta::Veta::ALL);
    ns_veta::Save(veta, "/home/csl/CppWorks/artwork/veta/output/veta.bin", ns_veta::Veta::ALL);
    ns_veta::Save(veta, "/home/csl/CppWorks/artwork/veta/output/veta.xml", ns_veta::Veta::ALL);
    return 0;
}