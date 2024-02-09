import os
import sys
import time
import click
import csv
import keyboard
import threading
import multiprocessing as mp
from typing import Any, List, Dict
from ctypes import c_float

import includes
import logs
from mqtt_client import MqttClient, MqttMsg
from app_mqtt import AnchorTelemetryData, IOS_TelemetryData

logger = logs.init_logger(__name__)

g_pause = threading.Event()
g_timestamp = 0.0

def _make_row_dict(row: List[str]) -> Dict[str, Any]:
    return {
        "type"          : str   (row[0]),
        "aid"           : int   (row[1]),
        "timestamp"     : float (row[2]),
        "status"        : int   (row[3]) if row[3] else None,
        "distance_m"    : float (row[4]) if row[4] else None,
        "azimuth_deg"   : float (row[5]) if row[5] else None,
        "elevation_deg" : float (row[6]) if row[6] else None,
        "los"           : int   (row[7]) if row[7] else None
    }

def keyboard_listener_thread():
    global g_pause
    global g_timestamp

    pause_time = 0.0

    while (1):
        if keyboard.is_pressed("space"):
            # toggle pause with spacebar

            if g_pause.is_set():
                g_timestamp += time.time() - pause_time
                g_pause.clear()

            else:
                pause_time = time.time()
                g_pause.set()

            # debounce
            time.sleep(0.2)

def print_timestamp_process(pipe):
    while (1):
        timestamp = pipe.recv()

        # print timestamp to 3 decimal places
        print(f"\r{timestamp:.3f}", end="")


@click.command()
@click.argument('csv_path', type=click.Path(exists=True))
@click.option('--uid', type=int, default=0, help='The UID of the iOS device to replay telemetry for.')
@click.option("--allow-pausing", is_flag=True, help="Allow pausing the replay with the spacebar, requires root.")
def cli(csv_path: str, uid: int, allow_pausing: bool) -> None:
    global g_pause
    global g_timestamp

    # check for root if pausing is enabled
    if allow_pausing and (os.geteuid() != 0):
        logger.error("pausing requires root!")
        sys.exit(1)

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

    # start processes and threads
    if allow_pausing:
        threading.Thread(target=keyboard_listener_thread, daemon=True).start()

    parent_conn, child_conn = mp.Pipe()

    p1 = mp.Process(target=print_timestamp_process, args=(child_conn,), daemon=True)
    p1.start()

    g_timestamp = time.time()

    for row in csv_data:
        # pause if spacebar is pressed
        while g_pause.is_set():
            time.sleep(0.1)

        # wait until it's time to send the message
        while (time.time() - g_timestamp) < float(row["timestamp"]):
            pass

        # send timestamp to the print process
        parent_conn.send( row["timestamp"] )

        # send the message
        if row["type"] == "anchor":
            timestamp   = int(row["timestamp"])*1000
            distance_mm = int( round(row["distance_m"] * 1000) )

            data = AnchorTelemetryData( row["status"],
                                        timestamp,
                                        distance_mm )

            mqtt_client.publish( f"gl/anchor/{row['aid']}/data", bytes(data) )

        elif row["type"] == "ios":
            data = IOS_TelemetryData( c_float(row["distance_m"]),
                                      int(row["azimuth_deg"]),
                                      int(row["elevation_deg"]),
                                      int(row["los"]) )

            mqtt_client.publish( f"gl/user/{uid}/data/anchor/{row['aid']}", bytes(data) )

        else:
            logger.error(f"unknown telemetry type: {row['type']}")
            continue

    # print_timestamp_process.kill()

    print()
    logger.info("Finished replaying telemetry data!")

def main():
    cli()

if __name__ == '__main__':
    main()
