# IMPORTANT: make sure to run with sudo if it doesn't work at first (lol)

import sys
import socket
import logging
from time import sleep
from zeroconf import IPVersion, ServiceInfo, Zeroconf

SERVICE_HOSTNAME = "GuidingLight._mqtt._tcp.local."

logger = logging.getLogger()
logging.basicConfig(level=logging.DEBUG)

def main():
    logger.info("Starting mDNS server...")

    info = ServiceInfo(
        "_mqtt._tcp.local.",
        SERVICE_HOSTNAME,
        addresses=[socket.gethostbyname(socket.gethostname())],
        port=1883
    )

    zeroconf = Zeroconf(ip_version=IPVersion.V4Only)
    zeroconf.register_service(info, ttl=128)

    logger.info(f"Registered service {SERVICE_HOSTNAME}")

    try:
        while True:
            pass

    finally:
        print()
        logger.info("Unregistering...")
        zeroconf.unregister_service(info)
        zeroconf.close()
        sys.exit(1)

if __name__ == '__main__':
    main()
