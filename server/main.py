import sys
import multiprocessing as mp

import logs

from gl_conf import GL_CONF, GL_CONF_JSON
from mqtt_client import MqttClient
from app_mqtt import SERVER_METADATA_TOPIC

from heartbeat_service import HeartbeatService
from outbound_data_service import OutboundDataService
from data_ingestion_service import DataIngestionService
from localization_service import LocalizationService
from pathfinding_service import PathfindingService

logger = logs.init_logger(__name__)

def publish_metadata():
    mqtt_client = MqttClient()

    mqtt_client.connect(GL_CONF.broker_address, GL_CONF.broker_port)
    mqtt_client.start_mainloop()

    # publish metadata as retained message
    mqtt_client.publish(SERVER_METADATA_TOPIC, GL_CONF_JSON, retain=True)

    logger.info("Metadata service exiting...")

    mqtt_client.disconnect()
    mqtt_client.stop_mainloop()


def main():
    # First thing, start the queued logs handler
    log_service = logs.LogQueueingService( in_conn=None, out_conn=None )
    log_service.start()

    logger.info("Starting rtls server...")

    # publish metadata to broker
    publish_metadata()

    try:
        # start outbound data service
        out = OutboundDataService( in_conn=None, out_conn=None )
        out.start()

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

        out.join()
        hb.join()
        dis.join()
        loc.join()
        pf.join()

        log_service.join()

        # should never get here (ideally)
        sys.exit(1)

    except KeyboardInterrupt:
        print("Terminating...")

        out.kill()
        hb.kill()

        dis.kill()
        loc.kill()
        pf.kill()

        log_service.kill()

        sys.exit(0)

if __name__ == '__main__':
    main()
