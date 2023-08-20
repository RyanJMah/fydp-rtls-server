import sys
import multiprocessing as mp

from data_ingestion_service import DataIngestionService
from localization_service import LocalizationService
from pathfinding_service import PathfindingService

def main():
    try:
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

        dis.join()
        loc.join()
        pf.join()

        # should never get here (ideally)
        sys.exit(1)

    except KeyboardInterrupt:
        print("Terminating...")

        dis.kill()
        loc.kill()
        pf.kill()

        sys.exit(0)

if __name__ == '__main__':
    main()
