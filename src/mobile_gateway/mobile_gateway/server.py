"""Bounded read-only HTTP server; put a TLS reverse proxy in front remotely."""
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import os
import socket
import threading


def token_from_environment():
    token = os.environ.get("HELIOS_GATEWAY_TOKEN", "")
    if len(token) < 32 or not token.isascii() or any(c.isspace() for c in token):
        raise ValueError("HELIOS_GATEWAY_TOKEN must contain at least 32 ASCII non-whitespace characters")
    return token


class TelemetryServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 16

    def __init__(self, address, state, token, max_clients=8, socket_timeout=3.0, request_timeout=5.0):
        if len(token) < 32 or not token.isascii() or any(c.isspace() for c in token):
            raise ValueError("A token of at least 32 ASCII non-whitespace characters is required")
        if max_clients < 1 or socket_timeout <= 0 or request_timeout <= 0:
            raise ValueError("Client limit and socket timeout must be positive")
        self.state = state
        self._token = token.encode("ascii")
        self._slots = threading.BoundedSemaphore(max_clients)
        self.socket_timeout = socket_timeout
        self.request_timeout = request_timeout
        super().__init__(address, TelemetryHandler)

    def process_request(self, request, client_address):
        request.settimeout(self.socket_timeout)
        if not self._slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self._slots.release()
            raise

    def process_request_thread(self, request, client_address):
        # An inactivity timeout alone does not bound clients that drip headers.
        def expire():
            try:
                request.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

        deadline = threading.Timer(self.request_timeout, expire)
        deadline.daemon = True
        deadline.start()
        try:
            super().process_request_thread(request, client_address)
        finally:
            deadline.cancel()
            self._slots.release()

    def handle_error(self, request, client_address):
        # Never print raw requests/headers or token-bearing exceptions.
        pass


class TelemetryHandler(BaseHTTPRequestHandler):
    server_version = "HeliosTelemetry/1"
    sys_version = ""

    def log_message(self, format, *args):
        # No request logging: query strings or malformed headers may contain secrets.
        pass

    def _reply(self, status, payload):
        body = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        if status == 401:
            self.send_header("WWW-Authenticate", 'Bearer realm="helios"')
        if status == 405:
            self.send_header("Allow", "GET")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)
        self.close_connection = True

    def _dispatch(self):
        if self.path != "/v1/telemetry":
            self._reply(404, {"error": "not_found"})
            return
        headers = self.headers.get_all("Authorization", [])
        provided = headers[0].encode("utf-8") if len(headers) == 1 else b""
        expected = b"Bearer " + self.server._token
        if not hmac.compare_digest(provided, expected):
            self._reply(401, {"error": "unauthorized"})
            return
        if self.command != "GET":
            self._reply(405, {"error": "method_not_allowed"})
            return
        self._reply(200, self.server.state.snapshot())

    # BaseHTTPRequestHandler normally emits 501 for unknown methods. All parsed
    # HTTP methods use the same explicit read-only dispatch instead.
    def __getattr__(self, name):
        if name.startswith("do_"):
            return self._dispatch
        raise AttributeError(name)


def arguments(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--behind-tls-proxy", action="store_true",
                        help="Acknowledge a secured TLS proxy and firewall protect a non-loopback listener")
    args, ros_args = parser.parse_known_args()
    try:
        loopback = ipaddress.ip_address(args.host).is_loopback
    except ValueError:
        parser.error("--host must be a numeric IPv4 bind address")
    if ":" in args.host:
        parser.error("Only IPv4 listeners are supported")
    if not loopback and not args.behind_tls_proxy:
        parser.error("Non-loopback binding requires --behind-tls-proxy and a configured TLS proxy/firewall")
    if not 0 <= args.port <= 65535:
        parser.error("--port must be between 0 and 65535")
    return args, ros_args
