import socket
import json
import multiprocessing as mp
from dataclasses import dataclass, asdict
from typing import Any

import logs
from gl_conf import GL_CONF
from abstract_service import AbstractService

logger = logs.init_logger(__name__)

g_queue = mp.Queue(-1)

@dataclass
class DebugEndpointData:
    tag: str
    data: Any

class DebugEndpointService(AbstractService):
    @staticmethod
    def push(data: DebugEndpointData):
        global g_queue
        g_queue.put(data)

    def accept_client(self):
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
                data = g_queue.get()

                json_payload = json.dumps( asdict(data) ).encode()

                payload_len = len(json_payload).to_bytes(4, "big")

                conn.sendall( payload_len + json_payload )

    def main(self, in_conn, out_conn):
        global g_queue

        if not GL_CONF.debug_endpoint.enabled:
            return

        while (1):
            try:
                self.accept_client()

            except Exception as e:
                logger.error(e)
                continue