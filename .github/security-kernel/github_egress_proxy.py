#!/usr/bin/env python3
"""Minimal fixed-destination CONNECT broker for the Security Kernel Git fetch zone."""
from __future__ import annotations
import re
import select
import socket
import socketserver
import sys

LISTEN_HOST = "0.0.0.0"
LISTEN_PORT = 3128
UPSTREAM_HOST = "github.com"
UPSTREAM_PORT = 443
MAX_HEADER_BYTES = 8192
MAX_HEADER_LINES = 32
CONNECT_TIMEOUT_SECONDS = 15
IDLE_TIMEOUT_SECONDS = 60
RECV_CHUNK_BYTES = 32768
REQUEST_LINE_RE = re.compile(rb"^CONNECT github\.com:443 HTTP/1\.[01]\r\n$")
HOST_HEADER_RE = re.compile(rb"^Host:\s*github\.com:443\r\n$", re.IGNORECASE)


def reject(client: socket.socket, status: bytes) -> None:
    payload = (
        b"HTTP/1.1 " + status + b"\r\n"
        b"Connection: close\r\n"
        b"Content-Length: 0\r\n"
        b"\r\n"
    )
    try:
        client.sendall(payload)
    except OSError:
        pass


def read_request(client: socket.socket) -> tuple[bytes, bytes]:
    client.settimeout(10)
    data = bytearray()
    while b"\r\n\r\n" not in data and len(data) < MAX_HEADER_BYTES:
        chunk = client.recv(4096)
        if not chunk:
            break
        data.extend(chunk)
    if len(data) > MAX_HEADER_BYTES:
        raise ValueError("header block exceeds bound")
    terminator = data.find(b"\r\n\r\n")
    if terminator < 0:
        raise ValueError("incomplete HTTP CONNECT header")
    header_end = terminator + 4
    return bytes(data[:header_end]), bytes(data[header_end:])


def validate_request(header: bytes) -> None:
    lines = header.split(b"\r\n")
    if not lines or lines[-1] or len(lines) > MAX_HEADER_LINES + 2:
        raise ValueError("malformed HTTP header framing")
    if not REQUEST_LINE_RE.fullmatch(lines[0] + b"\r\n"):
        raise ValueError("only CONNECT github.com:443 HTTP/1.x is permitted")
    host_headers = 0
    for raw in lines[1:-2]:
        if not raw:
            raise ValueError("unexpected blank header line")
        lowered = raw.lower()
        if lowered.startswith(b"proxy-authorization:"):
            raise ValueError("proxy authentication is forbidden")
        if lowered.startswith(b"content-length:") or lowered.startswith(b"transfer-encoding:"):
            raise ValueError("CONNECT request bodies are forbidden")
        if lowered.startswith(b"host:"):
            if not HOST_HEADER_RE.fullmatch(raw + b"\r\n"):
                raise ValueError("Host header must be exactly github.com:443")
            host_headers += 1
    if host_headers > 1:
        raise ValueError("duplicate Host header")


def tunnel(client: socket.socket, upstream: socket.socket, initial_payload: bytes) -> None:
    if initial_payload:
        upstream.sendall(initial_payload)
    sockets = (client, upstream)
    for item in sockets:
        item.setblocking(False)
    while True:
        readable, _, exceptional = select.select(list(sockets), [], list(sockets), IDLE_TIMEOUT_SECONDS)
        if exceptional or not readable:
            return
        for source in readable:
            try:
                payload = source.recv(RECV_CHUNK_BYTES)
            except BlockingIOError:
                continue
            if not payload:
                return
            target = upstream if source is client else client
            target.sendall(payload)


class Broker(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        client = self.request
        try:
            header, initial_payload = read_request(client)
            validate_request(header)
        except (OSError, ValueError):
            reject(client, b"403 Forbidden")
            return

        try:
            upstream = socket.create_connection(
                (UPSTREAM_HOST, UPSTREAM_PORT),
                timeout=CONNECT_TIMEOUT_SECONDS,
            )
        except OSError:
            reject(client, b"502 Bad Gateway")
            return

        try:
            client.sendall(
                b"HTTP/1.1 200 Connection Established\r\n"
                b"Connection: close\r\n"
                b"\r\n"
            )
            tunnel(client, upstream, initial_payload)
        finally:
            try:
                upstream.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            upstream.close()


class Server(socketserver.TCPServer):
    allow_reuse_address = False
    request_queue_size = 16


def main() -> int:
    if len(sys.argv) != 1:
        print("usage: github_egress_proxy.py", file=sys.stderr)
        return 2
    with Server((LISTEN_HOST, LISTEN_PORT), Broker) as server:
        server.serve_forever(poll_interval=0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
