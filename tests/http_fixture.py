#!/usr/bin/env python3
"""Deterministic HTTP/1.1 fixture; never contacts an external tile provider."""
import collections
import http.server
import pathlib
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.parse


class Server(http.server.ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def get_request(self):
        connection, address = super().get_request()
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return connection, address


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    counts = collections.Counter()
    lock = threading.Lock()

    def log_message(self, *_args):
        pass

    def do_GET(self):
        path = urllib.parse.urlsplit(self.path).path
        if path == "/count":
            target = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)["path"][0]
            with self.lock:
                body = str(self.counts[target]).encode()
            self.reply(200, body)
            return
        with self.lock:
            self.counts[path] += 1
            attempt = self.counts[path]
        body = ("tile:" + path).encode()
        if path.startswith("/slow/"):
            time.sleep(0.30)
        if path.startswith("/delay/"):
            time.sleep(5)
        if path.startswith("/permanent/"):
            self.reply(404, b"missing")
        elif path.startswith("/large-retry/"):
            self.reply(503, b"x" * (128 * 1024), retry_after="120")
        elif path.startswith("/long-retry/"):
            self.reply(429, b"busy", retry_after="120")
        elif path.startswith("/retry/") and attempt == 1:
            self.reply(503, b"busy", retry_after="1")
        elif path.startswith("/corrupt/") and attempt == 1:
            self.reply(200, b"<html>upstream failure</html>")
        elif path.startswith("/truncated/") and attempt == 1:
            self.send_response(200)
            self.send_header("Content-Length", str(len(body) + 50))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(body)
            self.close_connection = True
        elif path.startswith("/oversize/"):
            self.reply(200, b"x" * 8192)
        else:
            self.reply(200, body)

    def reply(self, status, body, retry_after=None):
        self.send_response(status)
        self.send_header("Content-Length", str(len(body)))
        if retry_after:
            self.send_header("Retry-After", retry_after)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass


if __name__ == "__main__":
    server = Server(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with tempfile.TemporaryDirectory(prefix="terrain-http-test-") as cache:
            result = subprocess.run([str(pathlib.Path(sys.argv[1]).resolve()),
                                     f"http://127.0.0.1:{server.server_port}", cache], timeout=45)
        raise SystemExit(result.returncode)
    finally:
        server.shutdown()
        server.server_close()
