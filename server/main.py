import sys
import multiprocessing as mp

import logs
from heartbeat_service import HeartbeatService
from debug_endpoint_service import DebugEndpointService
from data_ingestion_service import DataIngestionService
from localization_service import LocalizationService
from pathfinding_service import PathfindingService

logger = logs.init_logger(__name__)

def main():
    # First thing, start the queued logs handler
    log_service = logs.LogQueueingService( in_conn=None, out_conn=None )
    log_service.start()

    logger.info("Starting rtls server...")

    try:
        # start debug endpoint
        debug_endpoint = DebugEndpointService(in_conn=None, out_conn=None)
        debug_endpoint.start()

        # start heartbeat service
        hb = HeartbeatService( in_conn=None, out_conn=None )
        hb.start()

        # start application services
        dis_to_loc_conn_1, dis_to_loc_conn_2 = mp.Pipe()
        loc_to_pf_conn_1, loc_to_pf_conn_2 = mp.Pipe()

        dis = DataIngestionService( in_conn=None,
                                    out_conn=dis_to_loc_conn_1 )

        loc = LocalizationService( in_conn=dis_to_loc_conn_2,
                                   out_conn=loc_to_pf_conn_1 )

        pf = PathfindingService( in_conn=loc_to_pf_conn_2,
                                 out_conn=None )

        dis.start()
        loc.start()
        pf.start()

        debug_endpoint.join()
        hb.join()
        dis.join()
        loc.join()
        pf.join()

        log_service.join()

        # should never get here (ideally)
        sys.exit(1)

    except KeyboardInterrupt:
        print("Terminating...")

        debug_endpoint.kill()
        hb.kill()

        dis.kill()
        loc.kill()
        pf.kill()

        log_service.kill()

        sys.exit(0)

if __name__ == '__main__':
    main()
