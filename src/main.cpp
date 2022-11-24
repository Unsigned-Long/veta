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
        auto view = ns_veta::View::Create(ns_veta::UndefinedTimeT, 0, 0, 0, 200, 100);
        veta.views.insert(std::make_pair(view->viewId, view));
    }
    {
        // pose
        auto pose = ns_veta::Pose(ns_veta::Mat3d::Identity(), ns_veta::Vec3d::Zero());
        veta.poses.insert(std::make_pair(0, pose));
    }
    {
        // intri
        auto intri = ns_veta::PinholeIntrinsicBrownT2::Create(200, 100, 160, 140, 100, 50);
        veta.intrinsics.insert(std::make_pair(0, intri));
    }
    {
        // structure
        auto landmark = ns_veta::Landmark{ns_veta::Vec3d::Zero(), {}};
        landmark.obs.insert(std::make_pair(0, ns_veta::Observation(ns_veta::Vec2d::Zero(), 0)));
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