#include "decimator.h"

Decimator::Decimator()
{

}

void Decimator::attach(TopologicalMesh *mesh)
{
   m=mesh;

}

int Decimator::decimate(int collapses,int type)
{
    DecimaterT<TopologicalMesh> Dec(*m);


    ModQuadricT<TopologicalMesh>::Handle MQ;
//   Decimater::ModQuadricT<TopologicalMesh> Quad(*m);
//   Quad.initialize();
//   Quad.

    Dec.add(MQ);
    Dec.module(MQ).unset_max_err();
    if(!Dec.initialize()) return -1;

    Dec.setquad(Dec.module(MQ).quadricsHandle());

    return Dec.decimate(0);
}

void Decimator::detach()
{

}
