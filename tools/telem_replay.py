import sys
import time
import click
import csv
from typing import Any, List, Dict

import includes
import logs
from mqtt_client import MqttClient, MqttMsg
from app_mqtt import AnchorTelemetryData, IOS_TelemetryData

logger = logs.init_logger(__name__)

def _make_row_dict(row: List[str]) -> Dict[str, Any]:
    return {
        "type"          : str   (row[0]),
        "aid"           : int   (row[1]),
        "timestamp"     : float (row[2]),
        "status"        : int   (row[3]) if row[3] else None,
        "distance_m"    : float (row[4]) if row[4] else None,
        "azimuth_deg"   : float (row[5]) if row[5] else None,
        "elevation_deg" : float (row[6]) if row[6] else None,
        "los"           : bool  (row[7]) if row[7] else None
    }

@click.command()
@click.argument('csv_path', type=click.Path(exists=True))
@click.option('--uid', type=int, default=0, help='The UID of the iOS device to replay telemetry for.')
@click.option('--verbose', is_flag=True, help='Enable verbose logging.')
def cli(csv_path: str, uid: int, verbose: bool) -> None:
    csv_data: List[ Dict[str, Any] ]

    with open(csv_path, 'r') as csv_file:
        reader = csv.reader(csv_file, skipinitialspace=True)

        # skip header
        next(reader)

        csv_data = [ _make_row_dict(row) for row in reader ]
        
    # sort by timestamp
    csv_data.sort( key=lambda x: float(x["timestamp"]) )

    # connect to MQTT broker
    mqtt_client = MqttClient()
    mqtt_client.connect("localhost", 1883)
    mqtt_client.start_mainloop()

    logger.info("replaying telemetry data now...")

    start_time = time.time()

    for row in csv_data:
        # wait until it's time to send the message
        while (time.time() - start_time) < float(row["timestamp"]):
            pass

        if verbose:
            logger.info(f"replaying {row}")

        # send the message
        if row["type"] == "anchor":
            timestamp   = int(row["timestamp"])*1000
            distance_mm = int( round(row["distance_m"] * 1000) )

            data = AnchorTelemetryData( row["status"],
                                        timestamp,
                                        distance_mm )

            mqtt_client.publish( f"gl/anchor/{row['aid']}/data", bytes(data) )

        elif row["type"] == "ios":
            data = IOS_TelemetryData( row["distance_m"],
                                      row["azimuth_deg"],
                                      row["elevation_deg"],
                                      row["los"] )

            mqtt_client.publish( f"gl/ios/{row['aid']}/data/{uid}", bytes(data) )

        else:
            logger.error(f"unknown telemetry type: {row['type']}")
            continue

    logger.info("done replaying telemetry data!")

def main():
    cli()

if __name__ == '__main__':
    main()
