#!/usr/bin/env python3
"""Run the real terrain runtime against local raster tiles and repeated outages."""
import argparse
import collections
import http.server
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import threading
import time
import zlib


def png(rgb):
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))
    rows = b"".join(b"\0" + bytes(rgb) * 256 for _ in range(256))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 256, 256, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))


class Server(http.server.ThreadingHTTPServer):
    daemon_threads = True


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    started = time.monotonic()
    counts = collections.Counter()
    lock = threading.Lock()
    pngs = {i: png((i, i, i)) for i in range(100, 120)}
    dem = png((128, 20, 0))  # Exactly 20 metres in Terrarium encoding.

    def log_message(self, *_args):
        pass

    def do_GET(self):
        with self.lock:
            self.counts["requests"] += 1
            ordinal = self.counts["requests"]
        elapsed = time.monotonic() - self.started
        # Every 20 seconds, a two-second provider outage; first attempt of each
        # 37th request also fails, ensuring short runs test retry behaviour.
        outage = elapsed % 20 >= 18 or ordinal % 37 == 0
        if outage:
            with self.lock:
                self.counts["outage_responses"] += 1
            body, status = b"temporarily unavailable", 503
        elif self.path.startswith("/dem/"):
            body, status = self.dem, 200
        else:
            tile_x = int(self.path.split("/")[-2])
            body, status = self.pngs[100 + tile_x % 20], 200
        self.send_response(status)
        self.send_header("Content-Length", str(len(body)))
        if outage:
            self.send_header("Retry-After", "1")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    server = Server(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    try:
        with tempfile.TemporaryDirectory(prefix="terrain-streaming-soak-") as cache:
            result = subprocess.run([str(args.binary.resolve()), f"http://127.0.0.1:{server.server_port}",
                                     str(args.seconds), cache, str(args.report.resolve())], timeout=args.seconds + 60)
        if result.returncode:
            raise SystemExit(result.returncode)
        data = json.loads(args.report.read_text())
        with Handler.lock:
            data.update(Handler.counts)
        data["fixture"] = "local 256px PNG / two-second outage every twenty seconds"
        args.report.write_text(json.dumps(data, indent=2) + "\n")
        print(json.dumps(data))
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
