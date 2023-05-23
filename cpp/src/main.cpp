#include <iostream>
#include <stdint.h>
#include "Recast.h"
#include "InputGeom.h"
#include "Sample_TempObstacles.h"

int main();

int main()
{
    rcContext ctx;

    InputGeom* input_geom = new InputGeom();
    input_geom->load(
                &ctx,
                "/Users/ryanmah/main/fydp/rtls-server/recastnavigation/RecastDemo/Bin/Meshes/out.obj" );

    Sample_TempObstacles* pls_work = new Sample_TempObstacles();
    pls_work->setContext( &ctx );
    pls_work->resetCommonSettings();
    pls_work->handleMeshChanged( input_geom );
    pls_work->handleSettings();

    pls_work->handleBuild();

    pls_work->save("asdfasdf.idk");

    delete input_geom;
    delete pls_work;

    return 0;
}

