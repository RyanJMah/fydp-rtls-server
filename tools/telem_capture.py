import sys
import time
import datetime
import csv
import queue
import atexit
from typing import Any

import includes
import logs
from mqtt_client import MqttClient, MqttMsg
from app_mqtt import AnchorTelemetryData, IOS_TelemetryData

logger = logs.init_logger(__name__)

g_csv_file    : Any         = None
g_start_time  : float       = time.time()
g_write_queue : queue.Queue = queue.Queue()

# Filename format: telem_capture_YYYY-MM-DD_HH:MM:SS.csv
CSV_FILENAME = f"telem_capture_{ datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S') }.csv"
CSV_HEADER   = [
    "type",
    "aid",
    "timestamp",
    "status",
    "distance_m",
    "azimuth_deg",
    "elevation_deg",
    "los"
]


def anchor_data_handler(client: MqttClient, msg: MqttMsg, aid: int):
    global g_csv_writer
    global g_start_time

    data: AnchorTelemetryData

    try:
        data = AnchorTelemetryData.from_buffer_copy(msg.payload)
    except Exception as e:
        logger.error(f"anchor_data_handler: {e}")
        return

    g_write_queue.put(
        [
            "anchor",                                   # type
            aid,                                        # aid
            round( time.time() - g_start_time, 3 ),     # timestamp
            data.status,                                # status
            data.distance_mm / 1000,                    # distance_m
            None,                                       # azimuth_deg
            None,                                       # elevation_deg
            None                                        # los
        ]
    )

def ios_data_handler(client: MqttClient, msg: MqttMsg, uid: int, aid:int):
    global g_csv_writer
    global g_start_time

    data: IOS_TelemetryData

    try:
        data = IOS_TelemetryData.from_buffer_copy(msg.payload)
    except Exception as e:
        logger.error(f"ios_data_handler: {e}")
        return

    g_write_queue.put(
        [
            "ios",                                          # type
            aid,                                            # aid
            round( time.time() - g_start_time, 3 ),         # timestamp
            None,                                           # status
            data.distance_m,                                # distance_m
            data.azimuth_deg,                               # azimuth_deg
            data.elevation_deg,                             # elevation_deg
            data.los                                        # los
        ]
    )

def main():
    mqtt_client = MqttClient()

    mqtt_client.connect("localhost", 1883)

    mqtt_client.subscribe("gl/anchor/<id>/data", anchor_data_handler)
    mqtt_client.subscribe("gl/user/<uid>/data/<aid>", ios_data_handler)

    g_csv_file = open(CSV_FILENAME, "w")

    csv_writer = csv.writer( g_csv_file,
                             delimiter=',',
                             quotechar='"',
                             quoting=csv.QUOTE_MINIMAL )

    csv_writer.writerow(CSV_HEADER)

    mqtt_client.start_mainloop()

    while (1):
        row = g_write_queue.get()
        csv_writer.writerow(row)

def exit_handler():
    global g_csv_file

    # flush the queue
    while not g_write_queue.empty():
        row = g_write_queue.get()
        csv_writer.writerow(row)

    # close the file
    if g_csv_file:
        g_csv_file.close()

if __name__ == '__main__':
    atexit.register(exit_handler)

    try:
        main()
    except KeyboardInterrupt:
        exit_handler()
        sys.exit(0)

