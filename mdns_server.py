import socket
import logging
from time import sleep
from zeroconf import IPVersion, ServiceInfo, Zeroconf

import server.logs

SERVICE_HOSTNAME = "GuidingLight._mqtt._tcp.local."

logger = server.logs.init_logger(__name__)

def main():
    logger.setLevel(logging.DEBUG)

    info = ServiceInfo(
        "_mqtt._tcp.local.",
        SERVICE_HOSTNAME,
        addresses=[socket.inet_aton("1.2.3.4")],
        port=80,
    )

    zeroconf = Zeroconf(ip_version=IPVersion.V4Only)
    zeroconf.register_service(info)

    logger.info(f"Registered service {SERVICE_HOSTNAME}")

    try:
        while True:
            sleep(0.1)

    finally:
        logger.info("Unregistering...")
        zeroconf.unregister_service(info)
        zeroconf.close()

if __name__ == '__main__':
    main()
