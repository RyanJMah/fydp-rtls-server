# IMPORTANT: make sure to run with sudo if it doesn't work at first (lol)

import socket
from time import sleep
from zeroconf import IPVersion, ServiceInfo, Zeroconf

SERVICE_HOSTNAME = "GuidingLight._mqtt._tcp.local."

def main():
    print("Starting mDNS server...")

    info = ServiceInfo(
        "_mqtt._tcp.local.",
        SERVICE_HOSTNAME,
        addresses=[socket.gethostbyname(socket.gethostname())],
        port=80,
    )

    zeroconf = Zeroconf(ip_version=IPVersion.V4Only)
    zeroconf.register_service(info)

    print(f"Registered service {SERVICE_HOSTNAME}")

    try:
        while True:
            sleep(0.1)

    finally:
        print("Unregistering...")
        zeroconf.unregister_service(info)
        zeroconf.close()

if __name__ == '__main__':
    main()
