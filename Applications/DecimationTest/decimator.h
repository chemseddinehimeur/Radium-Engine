#ifndef DECIMATOR_H
#define DECIMATOR_H
#include <Core/Geometry/MeshPrimitives.hpp>
#include <Core/Geometry/TopologicalMesh.hpp>
#include <Core/Utils/Log.hpp>
#include <IO/deprecated/OBJFileManager.hpp>
#include <ModQuadricT.h>
#include <memory>
#include <decimatert.h>

using namespace OpenMesh;
using namespace Ra::Core::Geometry;

class Decimator
{
public:
    Decimator();
    void attach(TopologicalMesh *mesh);
    int decimate(int collapses,int type);
    void detach();
    TopologicalMesh *m;
};

#endif // DECIMATOR_H
