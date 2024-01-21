import socket
import threading
import multiprocessing as mp
import orjson as json   # fast json library
from dataclasses import dataclass, asdict
from typing import Any

import logs
import app_mqtt
from gl_conf import GL_CONF
from abstract_service import AbstractService
from mqtt_client import MqttClient, MqttMsg

logger = logs.init_logger(__name__)

g_debug_queue = mp.Queue(-1)
g_app_out_queue = mp.Queue(-1)

@dataclass
class OutboundData:
    tag: str
    data: Any

class OutboundDataService(AbstractService):
    @staticmethod
    def push(data: OutboundData, is_debug_data: bool = False):
        global g_debug_queue, g_app_out_queue

        if is_debug_data and GL_CONF.debug_endpoint.enabled:
            g_debug_queue.put(data)

        else:
            g_app_out_queue.put(data)


    def _debug_endpoint_accept_client(self):
        global g_debug_queue

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # this is so we can restart the program without waiting for the socket to timeout
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        sock.bind( (GL_CONF.debug_endpoint.host, GL_CONF.debug_endpoint.port) )
        sock.listen()

        logger.info("Debug endpoint listening on %s:%d" % (GL_CONF.debug_endpoint.host, GL_CONF.debug_endpoint.port))

        conn, addr = sock.accept()
        with conn:
            logger.info("client connected to debug endpoint...")

            while (1):
                data = g_debug_queue.get()

                json_payload = json.dumps(data, option=json.OPT_SERIALIZE_NUMPY)

                payload_len = len(json_payload).to_bytes(4, "big")

                conn.sendall( payload_len + json_payload )

    def debug_endpoint_thread(self):
        if not GL_CONF.debug_endpoint.enabled:
            return

        while (1):
            try:
                self._debug_endpoint_accept_client()

            except Exception as e:
                logger.error(e)
                continue


    def app_out_thread(self):
        logger.info("Starting outbound application data thread...")

        mqtt_client = MqttClient()
        mqtt_client.connect(GL_CONF.broker_address, GL_CONF.broker_port)

        mqtt_client.start_mainloop()

        # FIXME: hardcoded uid
        topic_base = app_mqtt.insert_uid(app_mqtt.IOS_BASE_TOPIC, "69")

        while (1):
            data = g_app_out_queue.get()
            data = asdict(data)

            tag = data["tag"]
            data = data["data"]

            json_payload = json.dumps(data, option=json.OPT_SERIALIZE_NUMPY)
            topic = topic_base + "/" + tag

            mqtt_client.publish(topic, json_payload)


    def main(self, in_conn, out_conn):
        t1 = threading.Thread(target=self.debug_endpoint_thread, daemon=True)
        t2 = threading.Thread(target=self.app_out_thread,        daemon=True)

        t1.start()
        t2.start()

        t1.join()
        t2.join()