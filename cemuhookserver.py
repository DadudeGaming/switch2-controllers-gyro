import socket, struct, time, zlib, os, threading, random
import controller

_server_sock = None
_stop_server = threading.Event()

# SERVER_PORT = int(os.getenv("DSU_PORT", "26760"))
SERVER_PORT = 26760
PROTO_VER   = 1001

class _SensorState:
    def __init__(self):
        self._lock = threading.Lock()
        # defaults: device at rest, Z-up
        self._accel_g = (0.0, 0.0, 1.0)     # (ax, ay, az) in g
        self._gyro_dps = (0.0, 0.0, 0.0)    # (gx, gy, gz) in deg/s

    def set_motion(self, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps):
        with self._lock:
            self._accel_g = (float(ax_g), float(ay_g), float(az_g))
            self._gyro_dps = (float(gx_dps), float(gy_dps), float(gz_dps))

    def get_accel(self):
        with self._lock:
            return self._accel_g

    def get_gyro(self):
        with self._lock:
            return self._gyro_dps

_sensor = _SensorState()

# public API you can import from other files:
def update_motion(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps):
    """Thread-safe: push latest sensor values (g, deg/s) from your controller code."""
    _sensor.set_motion(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps)

# ---- Replace these with your real sensor reads ----
def read_accel():  # return (ax, ay, az) in g's
    # Example: return your controller's accel normalized to g
    return _sensor.get_accel()

def read_gyro():   # return (pitch, yaw, roll) in deg/s
    # Example: fill with your IMU's angular velocity (deg/s)
    return _sensor.get_gyro()
# ---------------------------------------------------

_last_ts_us = 0
_last_accel = (None, None, None)

def _should_bump_ts(ax, ay, az):
    global _last_accel
    la = _last_accel
    if la[0] is None:
        _last_accel = (ax, ay, az)
        return True
    # bump if accel changed by > 0.002 g on any axis (tune as you like)
    if (abs(ax - la[0]) > 0.002 or abs(ay - la[1]) > 0.002 or abs(az - la[2]) > 0.002):
        _last_accel = (ax, ay, az)
        return True
    return False

def crc32_le(b):
    # Little-endian CRC32 of the whole packet with the CRC field zeroed
    return zlib.crc32(b) & 0xFFFFFFFF

def pack_header(magic, msg_type, payload, client_id):
    # DSU header (16B) + msg_type (4B) -> then payload
    #  0:4  magic; 4:2 proto; 6:2 len(payload + msg_type)
    #  8:4  crc32 (filled later); 12:4 sender id
    length_wo_header = 4 + len(payload)  # msg_type + payload
    hdr = bytearray()
    hdr += magic            # b"DSUS" (server) or b"DSUC" (client)
    hdr += struct.pack("<H", PROTO_VER)
    hdr += struct.pack("<H", length_wo_header)
    hdr += b"\x00\x00\x00\x00"          # CRC placeholder
    hdr += struct.pack("<I", client_id) # sender id
    hdr += struct.pack("<I", msg_type)  # not part of 16B header; counts toward len
    pkt = bytes(hdr) + payload

    # compute CRC with CRC field zeroed
    pkt = bytearray(pkt)
    pkt[8:12] = b"\x00\x00\x00\x00"
    crc = crc32_le(pkt)
    pkt[8:12] = struct.pack("<I", crc)
    return bytes(pkt)

def controller_id_block():
    # “Shared beginning” (11 bytes): slot, state, model, conn, mac(6), battery
    slot = 0                      # we expose slot 0
    state = 2                     # 2=connected
    model = 2                     # 2=full gyro
    conn  = 2                     # 2=bluetooth (or 1=USB, doesn't matter much)
    mac   = b"\x12\x34\x56\x78\x9A\xBC"  # any stable 6 bytes
    batt  = 0x04                  # “High”
    return struct.pack("<BBBB6sB", slot, state, model, conn, mac, batt)

def build_info_response(client_id):
    # Respond to 0x100001 (“connected controllers info”)
    # Payload: [11B shared] + 1 zero byte
    payload = controller_id_block() + b"\x00"
    return pack_header(b"DSUS", 0x100001, payload, client_id)

# ---- Replace these with your real sensor reads ----
def read_accel():  # return (ax, ay, az) in g's
    return _sensor.get_accel()

def read_gyro():   # return (pitch, yaw, roll) in deg/s
    return _sensor.get_gyro()
# ---------------------------------------------------

def build_data_packet(packet_num, client_id):
    shared = controller_id_block()
    connected = 1
    buttons1 = 0
    buttons2 = 0
    home     = 0
    touchbtn = 0
    lstx=lsty=rstx=rsty=128
    analog = bytes([0]*12)
    touch = bytes([0]*12)

    ax, ay, az = read_accel()
    gp, gy, gr = read_gyro()

    # Always use current time (µs)
    ts_us = int(time.time() * 1_000_000)

    payload = bytearray()
    payload += shared
    payload += struct.pack("<B", connected)
    payload += struct.pack("<I", packet_num)
    payload += struct.pack("<B", buttons1)
    payload += struct.pack("<B", buttons2)
    payload += struct.pack("<B", home)
    payload += struct.pack("<B", touchbtn)
    payload += struct.pack("<BBBB", lstx, lsty, rstx, rsty)
    payload += analog
    payload += touch
    payload += struct.pack("<Q", ts_us)
    payload += struct.pack("<f", ax)
    payload += struct.pack("<f", ay)
    payload += struct.pack("<f", az)
    payload += struct.pack("<f", gp)  # pitch (deg/s)
    payload += struct.pack("<f", gy)  # yaw   (deg/s)
    payload += struct.pack("<f", gr)  # roll  (deg/s)
    return pack_header(b"DSUS", 0x100002, bytes(payload), client_id)

def stream_thread(sock, addr, client_id, stop_event):
    pkt = 0
    period = 1.0 / 120.0
    while not stop_event.is_set():
        pkt += 1
        try:
            data = build_data_packet(pkt, client_id)
            sock.sendto(data, addr)
        except (ConnectionResetError, OSError):
            # Peer closed / ICMP Port Unreachable / network hiccup; stop this stream
            break
        time.sleep(period)


def run_server():
    global _server_sock, _stop_server
    _stop_server.clear()

    import sys, struct as _struct, socket as _s
    sock = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
    sock.setsockopt(_s.SOL_SOCKET, _s.SO_REUSEADDR, 1)
    _server_sock = sock

    sock.bind(("0.0.0.0", SERVER_PORT))
    print(f"DSU server listening on 0.0.0.0:{SERVER_PORT}")

    # Windows-only: ignore UDP "connection reset" ICMPs
    try:
        if sys.platform.startswith("win") and hasattr(_s, "SIO_UDP_CONNRESET"):
            # 0 = disable the error -> ignore ICMP Port Unreachable
            sock.ioctl(_s.SIO_UDP_CONNRESET, _struct.pack("I", 0))
    except Exception:
        # Harmless if unavailable
        pass

    streams = {}  # key: (ip,port) -> (thread, stop_event, client_id)
    my_id = random.getrandbits(32)

    while not _stop_server.is_set():
        try:
            data, addr = sock.recvfrom(2048)
        except ConnectionResetError:
            # Windows ICMP "Port Unreachable" — safe to ignore
            continue
        except OSError:
            break  # socket closed while waiting

        if len(data) < 20:
            continue

        magic = data[0:4]
        if magic != b"DSUC":  # only react to client messages
            continue

        client_id = struct.unpack("<I", data[12:16])[0]
        msg_type  = struct.unpack("<I", data[16:20])[0]

        if msg_type == 0x100001:
            resp = build_info_response(my_id)
            sock.sendto(resp, addr)

        elif msg_type == 0x100002:
            # Start/refresh streaming for this client
            if addr in streams:
                t, ev, _ = streams.pop(addr)
                ev.set()
                t.join(timeout=0.1)
            stop = threading.Event()
            t = threading.Thread(target=stream_thread, args=(sock, addr, my_id, stop), daemon=True)
            streams[addr] = (t, stop, my_id)
            t.start()

        elif msg_type == 0x100000:
            payload = struct.pack("<H", PROTO_VER)
            resp = pack_header(b"DSUS", 0x100000, payload, my_id)
            sock.sendto(resp, addr)
    
    sock.close()
    print("Server closed.")

def close_server():
    """Stop the DSU server gracefully from external code."""
    global _server_sock, _stop_server
    _stop_server.set()
    if _server_sock:
        try:
            _server_sock.close()
        except Exception:
            pass
        _server_sock = None
    print("Close signal sent to server.")

if __name__ == "__main__":
    run_server()
