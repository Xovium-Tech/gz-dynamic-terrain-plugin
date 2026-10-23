#!/usr/bin/env python3
"""Trust-store and hostname checks against an ephemeral local HTTPS server."""
import os
from pathlib import Path
import ssl
import subprocess
import sys
import tempfile
import threading
from http_fixture import Handler, Server


with tempfile.TemporaryDirectory(prefix="terrain-tls-test-") as temporary:
    root = Path(temporary)
    key, cert = root / "key.pem", root / "cert.pem"
    subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
                    "-days", "1", "-keyout", str(key), "-out", str(cert),
                    "-subj", "/CN=localhost", "-addext", "subjectAltName=DNS:localhost"],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    server = Server(("127.0.0.1", 0), Handler)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(cert, key)
    server.socket = context.wrap_socket(server.socket, server_side=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    env = dict(os.environ, NO_PROXY="localhost,127.0.0.1", no_proxy="localhost,127.0.0.1")
    env.pop("CURL_CA_BUNDLE", None)
    env.pop("SSL_CERT_FILE", None)
    cases = [
        ("untrusted", "localhost", {}, False),
        ("trusted", "localhost", {"CURL_CA_BUNDLE": str(cert)}, True),
        ("wrong-host", "127.0.0.1", {"CURL_CA_BUNDLE": str(cert)}, False),
        ("missing-ca", "localhost", {"CURL_CA_BUNDLE": str(root / "missing.pem")}, False),
        ("ssl-cert-file", "localhost", {"SSL_CERT_FILE": str(cert)}, True),
        ("override-precedence", "localhost", {"SSL_CERT_FILE": str(cert),
                                              "CURL_CA_BUNDLE": str(root / "missing.pem")}, False),
    ]
    try:
        for name, host, overrides, expected in cases:
            cache = root / name
            result = subprocess.run([str(Path(sys.argv[1]).resolve()),
                                     f"https://{host}:{server.server_port}", str(cache), "--child"],
                                    env=dict(env, **overrides), timeout=10)
            assert (result.returncode == 0) == expected, f"TLS case failed: {name}"
            assert (cache / "process").exists() == expected, f"TLS failure populated cache: {name}"
        print("TLS trust overrides, precedence, untrusted peers and hostname verification passed")
    finally:
        server.shutdown()
        server.server_close()
