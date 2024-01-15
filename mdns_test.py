import socket
import random

from mdns_server import SERVICE_HOSTNAME

def mdns_query(name: str):
    header = bytes([0x00, 0x00])

    question = bytearray()

    for label in name.split("."):
        question.append( len(label) )
        question += label.encode()

    # question += bytes([0x00])

    # 00 01 for A records
    question += bytes([0x00, 0x01])

    # UNICAST-RESPONSE=1, QCLASS=1
    question += ( (0 << 15) | 1 ).to_bytes(2, "big")

    trans_id = random.randint(0, 0xFFFF + 1)

    # -------------------------
    # |    transaction_id     |
    # -------------------------
    # |        header         |
    # -------------------------
    # | Questions:     00 01  |
    # -------------------------
    # | Answer RRs:    00 00  |
    # -------------------------
    # | Authority RRs:  00 00 |
    # -------------------------
    # | Additional RRs: 00 00 |
    # -------------------------
    # |       question        |
    # -------------------------

    pkt = trans_id.to_bytes(2, "big")  + \
          header                       + \
          bytes([0x00, 0x01])          + \
          bytes([0x00, 0x00])          + \
          bytes([0x00, 0x00])          + \
          bytes([0x00, 0x00])          + \
          question

    return pkt, trans_id

def main():
    query, trans_id = mdns_query(SERVICE_HOSTNAME)

    multicast_addr = "224.0.0.251"
    multicast_port = 5353

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    sock.sendto(query, (multicast_addr, multicast_port))

    while (1):
        print(sock.recv(1024))

if __name__ == "__main__":
    main()
