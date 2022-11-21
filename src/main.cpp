//
// Created by csl on 11/21/22.
//
#include "iostream"
#include "veta/veta.h"
#include "veta/camera/pinhole_eadial.h"

int main(int argc, char **argv) {
    ns_veta::Veta veta;

    ns_veta::Save(veta, "/home/csl/CppWorks/artwork/veta/output/veta.json", ns_veta::Veta::ALL);
    ns_veta::Save(veta, "/home/csl/CppWorks/artwork/veta/output/veta.bin", ns_veta::Veta::ALL);
    ns_veta::Save(veta, "/home/csl/CppWorks/artwork/veta/output/veta.xml", ns_veta::Veta::ALL);

    ns_veta::Load(veta, "/home/csl/CppWorks/artwork/veta/output/veta.json", ns_veta::Veta::ALL);
    ns_veta::Load(veta, "/home/csl/CppWorks/artwork/veta/output/veta.bin", ns_veta::Veta::ALL);
    ns_veta::Load(veta, "/home/csl/CppWorks/artwork/veta/output/veta.xml", ns_veta::Veta::ALL);
    return 0;
}