"""
Веб-дашборд телеметрії MAVLink.

Складається з двох частин:

1. Фоновий потік (reader_thread) читає MAVLink так само, як read_telemetry.py,
   але не друкує в консоль, а складає останні значення у словник STATE.
2. HTTP-сервер зі стандартної бібліотеки (без Flask, щоб не ставити нових
   залежностей на малинку) віддає сторінку index.html і той самий STATE
   у вигляді JSON за адресою /api/telemetry. Сторінка сама питає цей JSON
   кілька разів на секунду і перемальовує схему.

Запуск:
    source venv/bin/activate
    python3 src/dashboard.py                 # слухає udpin:0.0.0.0:14550
    python3 src/dashboard.py --demo          # без політника, синтетичні дані
    python3 src/dashboard.py --connect /dev/serial0 --baud 57600

Потім відкрити http://localhost:8080 (або http://<IP малинки>:8080 з ноутбука).
"""

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

STATIC_DIR = Path(__file__).parent / "static"

# Останні відомі значення. Пишеться з потоку-читача, читається з HTTP-потоків,
# тому будь-який доступ — під замком LOCK.
LOCK = threading.Lock()
STATE = {
    "connected": False,
    "armed": False,
    "mode": "—",
    "roll": 0.0,          # радіани
    "pitch": 0.0,
    "yaw": 0.0,
    "alt": 0.0,           # м над точкою зльоту
    "alt_msl": 0.0,       # м над рівнем моря
    "lat": None,
    "lon": None,
    "home_lat": None,
    "home_lon": None,
    "distance": 0.0,      # м по горизонталі від точки зльоту
    "groundspeed": 0.0,   # м/с
    "climb": 0.0,         # м/с, + вгору
    "heading": 0.0,       # градуси
    "battery_v": 0.0,
    "battery_pct": -1,
    "sats": 0,
    "fix": 0,
    "rc": [0, 0, 0, 0],
    "last_update": 0.0,
}


def haversine(lat1, lon1, lat2, lon2):
    """Відстань по поверхні між двома точками, метри."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def reader_thread(connect, baud):
    """Читає MAVLink у нескінченному циклі й оновлює STATE."""
    from pymavlink import mavutil

    while True:
        try:
            print(f"connecting to {connect} ...")
            m = mavutil.mavlink_connection(connect, baud=baud)
            m.wait_heartbeat()
            print("connected, sys", m.target_system)
            with LOCK:
                STATE["connected"] = True

            while True:
                msg = m.recv_match(blocking=True, timeout=5)
                if msg is None:
                    # 5 секунд тиші — вважаємо, що зв'язок пропав
                    with LOCK:
                        STATE["connected"] = False
                    continue

                t = msg.get_type()
                with LOCK:
                    STATE["connected"] = True
                    STATE["last_update"] = time.time()

                    if t == "ATTITUDE":
                        STATE["roll"] = msg.roll
                        STATE["pitch"] = msg.pitch
                        STATE["yaw"] = msg.yaw
                    elif t == "GLOBAL_POSITION_INT":
                        STATE["alt"] = msg.relative_alt / 1000.0
                        STATE["alt_msl"] = msg.alt / 1000.0
                        STATE["lat"] = msg.lat / 1e7
                        STATE["lon"] = msg.lon / 1e7
                        STATE["heading"] = msg.hdg / 100.0 if msg.hdg != 65535 else STATE["heading"]
                        # Якщо політник ще не прислав HOME_POSITION — беремо
                        # першу отриману точку за домівку.
                        if STATE["home_lat"] is None and msg.lat != 0:
                            STATE["home_lat"] = STATE["lat"]
                            STATE["home_lon"] = STATE["lon"]
                        if STATE["home_lat"] is not None:
                            STATE["distance"] = haversine(
                                STATE["home_lat"], STATE["home_lon"],
                                STATE["lat"], STATE["lon"])
                    elif t == "HOME_POSITION":
                        STATE["home_lat"] = msg.latitude / 1e7
                        STATE["home_lon"] = msg.longitude / 1e7
                    elif t == "VFR_HUD":
                        STATE["groundspeed"] = msg.groundspeed
                        STATE["climb"] = msg.climb
                    elif t == "SYS_STATUS":
                        STATE["battery_v"] = msg.voltage_battery / 1000.0
                        STATE["battery_pct"] = msg.battery_remaining
                    elif t == "GPS_RAW_INT":
                        STATE["sats"] = msg.satellites_visible
                        STATE["fix"] = msg.fix_type
                    elif t == "RC_CHANNELS":
                        STATE["rc"] = [msg.chan1_raw, msg.chan2_raw,
                                       msg.chan3_raw, msg.chan4_raw]
                    elif t == "HEARTBEAT":
                        STATE["armed"] = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        STATE["mode"] = mavutil.mode_string_v10(msg)
        except Exception as e:
            print("reader error:", e, "— retry in 3 s")
            with LOCK:
                STATE["connected"] = False
            time.sleep(3)


def demo_thread():
    """Синтетичний політ — щоб подивитись сторінку без SITL."""
    t0 = time.time()
    with LOCK:
        STATE["home_lat"], STATE["home_lon"] = 50.4501, 30.5234
    while True:
        t = time.time() - t0
        with LOCK:
            STATE["connected"] = True
            STATE["armed"] = True
            STATE["mode"] = "GUIDED"
            STATE["roll"] = 0.25 * math.sin(t / 3)
            STATE["pitch"] = 0.12 * math.sin(t / 5)
            STATE["yaw"] = (t / 8) % (2 * math.pi) - math.pi
            STATE["alt"] = 40 + 25 * math.sin(t / 7)
            STATE["alt_msl"] = STATE["alt"] + 180
            STATE["distance"] = 120 + 90 * math.sin(t / 11)
            STATE["groundspeed"] = 6 + 3 * math.sin(t / 4)
            STATE["climb"] = 25 * math.cos(t / 7) / 7
            STATE["heading"] = math.degrees(STATE["yaw"]) % 360
            STATE["battery_v"] = 12.4
            STATE["battery_pct"] = 78
            STATE["sats"], STATE["fix"] = 14, 3
            STATE["rc"] = [1500 + int(300 * math.sin(t / 3)), 1500, 1400, 1500]
            STATE["last_update"] = time.time()
        time.sleep(0.05)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/api/telemetry"):
            with LOCK:
                body = json.dumps(STATE).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        path = "index.html" if self.path in ("/", "") else self.path.lstrip("/")
        file = (STATIC_DIR / path).resolve()
        # Не даємо вийти за межі static/
        if not str(file).startswith(str(STATIC_DIR.resolve())) or not file.is_file():
            self.send_error(404)
            return
        body = file.read_bytes()
        ctype = {"html": "text/html; charset=utf-8",
                 "js": "text/javascript",
                 "css": "text/css"}.get(file.suffix.lstrip("."), "text/plain")
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass  # не засмічуємо консоль рядком на кожен запит


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--connect", default="udpin:0.0.0.0:14550")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--demo", action="store_true", help="синтетичні дані без політника")
    a = p.parse_args()

    target = demo_thread if a.demo else lambda: reader_thread(a.connect, a.baud)
    threading.Thread(target=target, daemon=True).start()

    print(f"dashboard: http://localhost:{a.port}   (Ctrl+C щоб зупинити)")
    ThreadingHTTPServer(("0.0.0.0", a.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
