from typing import Optional

import logs
from gl_conf import GL_CONF

import app_paths
import lib_pathfinding as cpp     # type: ignore

from abstract_service import AbstractService
from localization_service import LocalizationService_OutData
from debug_endpoint_service import DebugEndpointService, DebugEndpointData

logger = logs.init_logger(__name__)

class PathfindingService(AbstractService):
    def main(self, in_conn, out_conn):
        pf = cpp.Pathfinder()

        pf.load_navmesh( GL_CONF.navmesh_path )
        pf.set_navmesh_scale( GL_CONF.navmesh_to_real_life_scale )
    
        while (1):
            position_data: LocalizationService_OutData = in_conn.recv()     # type: ignore

            x = position_data.x
            y = position_data.y
            z = position_data.z

            path = pf.find_path( (250, 100, 0), (500, 500, 0)  )

            # Push to debug endpoint
            debug_data = DebugEndpointData( tag="path",
                                            data=path )

            DebugEndpointService.push(debug_data)
