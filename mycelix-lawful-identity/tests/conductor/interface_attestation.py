#!/usr/bin/env python3
"""Canonical authenticated-interface receipts for real Holochain campaigns.

The reviewed control executable records one fresh admin-interface and one fresh
app-interface round trip for every conductor during each runtime probe.  This
module validates their exact framing and recomputes all hashes; callers must not
trust hash strings supplied by the control process.
"""
from __future__ import annotations

import base64
import hashlib
import json
from typing import Any

INTERFACE_ATTESTATION_SCHEMA = "mycelix-holochain-interface-attestation-v1"
INTERFACE_BINDING_DOMAIN = "MYCELIX:HolochainInterfaceBinding:v1"
ADMIN_ROUND_TRIP_DOMAIN = "MYCELIX:HolochainAdminInterfaceRoundTrip:v1"
APP_ROUND_TRIP_DOMAIN = "MYCELIX:HolochainAppInterfaceRoundTrip:v1"
ATTESTATION_DOMAIN = "MYCELIX:HolochainInterfaceAttestation:v1"
BINDING_SET_DOMAIN = "MYCELIX:HolochainInterfaceBindingSet:v1"
ALLOWED_TRANSPORTS = {"unix", "loopback-tcp"}


class InterfaceAttestationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise InterfaceAttestationError(message)


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def sha256_b64(data: bytes) -> str:
    return base64.b64encode(hashlib.sha256(data).digest()).decode("ascii")


def require_hash32(value: Any, name: str) -> str:
    require(isinstance(value, str) and len(value) == 44, f"{name} must be canonical standard-base64 SHA-256")
    try:
        decoded = base64.b64decode(value, validate=True)
    except Exception as error:
        raise InterfaceAttestationError(f"{name} is not canonical base64: {error}") from error
    require(len(decoded) == 32 and base64.b64encode(decoded).decode("ascii") == value, f"{name} must decode to exactly 32 bytes")
    return value


def require_text(value: Any, name: str, maximum: int = 4096) -> str:
    require(isinstance(value, str) and value != "", f"{name} must be non-empty text")
    require(len(value.encode("utf-8")) <= maximum, f"{name} exceeds {maximum} bytes")
    require(not any(ord(ch) < 0x20 for ch in value), f"{name} contains control characters")
    return value


def interface_binding_hash(conductor: dict[str, Any]) -> str:
    return sha256_b64(canonical_json_bytes({
        "domain": INTERFACE_BINDING_DOMAIN,
        "conductor_id": require_text(conductor.get("id"), "conductor.id", 128),
        "agent_pub_key": require_text(conductor.get("agent_pub_key"), "conductor.agent_pub_key"),
        "admin_endpoint_binding_hash": require_hash32(conductor.get("admin_endpoint_binding_hash"), "admin_endpoint_binding_hash"),
        "app_endpoint_binding_hash": require_hash32(conductor.get("app_endpoint_binding_hash"), "app_endpoint_binding_hash"),
        "app_signer_pub_key": require_text(conductor.get("app_signer_pub_key"), "app_signer_pub_key"),
        "app_auth_token_hash": require_hash32(conductor.get("app_auth_token_hash"), "app_auth_token_hash"),
    }))


def interface_binding_set_hash(conductors: list[dict[str, Any]]) -> str:
    normalized = [
        {"id": require_text(item.get("id"), "conductor.id", 128), "interface_binding_hash": interface_binding_hash(item)}
        for item in sorted(conductors, key=lambda value: value.get("id", ""))
    ]
    require(len({item["id"] for item in normalized}) == len(normalized), "interface binding conductor ids repeat")
    return sha256_b64(canonical_json_bytes({"domain": BINDING_SET_DOMAIN, "conductors": normalized}))


def _round_trip_hash(domain: str, request_nonce: str, conductor_id: str, observed_at: int, receipt: dict[str, Any]) -> str:
    framed = {key: value for key, value in receipt.items() if key != "round_trip_hash"}
    return sha256_b64(canonical_json_bytes({
        "domain": domain,
        "request_nonce": request_nonce,
        "conductor_id": conductor_id,
        "observed_at_unix_micros": observed_at,
        "receipt": framed,
    }))


def _validate_admin(value: Any, *, request_nonce: str, conductor_id: str, observed_at: int, base: dict[str, Any], runtime: dict[str, Any]) -> dict[str, Any]:
    require(isinstance(value, dict), "admin interface receipt must be an object")
    expected = {
        "transport", "endpoint_binding_hash", "conductor_instance_id",
        "request_hash", "response_hash", "round_trip_hash",
    }
    require(set(value) == expected, "admin interface receipt has missing or unknown fields")
    transport = require_text(value.get("transport"), "admin.transport", 32)
    require(transport in ALLOWED_TRANSPORTS, "admin transport must be unix or loopback-tcp")
    endpoint = require_hash32(value.get("endpoint_binding_hash"), "admin.endpoint_binding_hash")
    require(endpoint == base["admin_endpoint_binding_hash"], "admin endpoint binding differs from reviewed configuration")
    instance = require_hash32(value.get("conductor_instance_id"), "admin.conductor_instance_id")
    require(instance == runtime["conductor_instance_id"], "admin receipt names a different conductor instance")
    request_hash = require_hash32(value.get("request_hash"), "admin.request_hash")
    response_hash = require_hash32(value.get("response_hash"), "admin.response_hash")
    round_trip = require_hash32(value.get("round_trip_hash"), "admin.round_trip_hash")
    normalized = {
        "transport": transport,
        "endpoint_binding_hash": endpoint,
        "conductor_instance_id": instance,
        "request_hash": request_hash,
        "response_hash": response_hash,
        "round_trip_hash": round_trip,
    }
    require(round_trip == _round_trip_hash(ADMIN_ROUND_TRIP_DOMAIN, request_nonce, conductor_id, observed_at, normalized), "admin round-trip hash mismatch")
    return normalized


def _validate_app(value: Any, *, request_nonce: str, conductor_id: str, observed_at: int, base: dict[str, Any], runtime: dict[str, Any]) -> dict[str, Any]:
    require(isinstance(value, dict), "app interface receipt must be an object")
    expected = {
        "transport", "endpoint_binding_hash", "conductor_instance_id", "cell_id_hash",
        "app_signer_pub_key", "app_auth_token_hash", "call_provenance_agent_pub_key",
        "request_hash", "response_hash", "round_trip_hash",
    }
    require(set(value) == expected, "app interface receipt has missing or unknown fields")
    transport = require_text(value.get("transport"), "app.transport", 32)
    require(transport in ALLOWED_TRANSPORTS, "app transport must be unix or loopback-tcp")
    endpoint = require_hash32(value.get("endpoint_binding_hash"), "app.endpoint_binding_hash")
    require(endpoint == base["app_endpoint_binding_hash"], "app endpoint binding differs from reviewed configuration")
    instance = require_hash32(value.get("conductor_instance_id"), "app.conductor_instance_id")
    require(instance == runtime["conductor_instance_id"], "app receipt names a different conductor instance")
    cell_id = require_hash32(value.get("cell_id_hash"), "app.cell_id_hash")
    require(cell_id == runtime["cell_id_hash"], "app receipt names a different cell")
    signer = require_text(value.get("app_signer_pub_key"), "app.app_signer_pub_key")
    require(signer == base["app_signer_pub_key"] == base["agent_pub_key"], "app signer is not the reviewed conductor agent")
    token_hash = require_hash32(value.get("app_auth_token_hash"), "app.app_auth_token_hash")
    require(token_hash == base["app_auth_token_hash"], "app authentication token hash differs from reviewed configuration")
    provenance = require_text(value.get("call_provenance_agent_pub_key"), "app.call_provenance_agent_pub_key")
    require(provenance == base["agent_pub_key"], "app call provenance differs from the conductor agent")
    request_hash = require_hash32(value.get("request_hash"), "app.request_hash")
    response_hash = require_hash32(value.get("response_hash"), "app.response_hash")
    round_trip = require_hash32(value.get("round_trip_hash"), "app.round_trip_hash")
    normalized = {
        "transport": transport,
        "endpoint_binding_hash": endpoint,
        "conductor_instance_id": instance,
        "cell_id_hash": cell_id,
        "app_signer_pub_key": signer,
        "app_auth_token_hash": token_hash,
        "call_provenance_agent_pub_key": provenance,
        "request_hash": request_hash,
        "response_hash": response_hash,
        "round_trip_hash": round_trip,
    }
    require(round_trip == _round_trip_hash(APP_ROUND_TRIP_DOMAIN, request_nonce, conductor_id, observed_at, normalized), "app round-trip hash mismatch")
    return normalized


def validate_interface_attestation(
    value: Any,
    *,
    request_nonce: str,
    conductor_id: str,
    observed_at_unix_micros: int,
    base_conductor: dict[str, Any],
    runtime_conductor: dict[str, Any],
) -> dict[str, Any]:
    require(isinstance(value, dict), "interface attestation must be an object")
    expected = {"schema", "request_nonce", "observed_at_unix_micros", "conductor_id", "admin", "app", "attestation_hash"}
    require(set(value) == expected, "interface attestation has missing or unknown fields")
    require(value.get("schema") == INTERFACE_ATTESTATION_SCHEMA, "unsupported interface attestation schema")
    require(require_hash32(value.get("request_nonce"), "interface request_nonce") == request_nonce, "interface attestation changed request nonce")
    require(value.get("observed_at_unix_micros") == observed_at_unix_micros, "interface attestation timestamp differs from deployment probe")
    require(value.get("conductor_id") == conductor_id, "interface attestation names a different conductor")
    binding_hash = interface_binding_hash(base_conductor)
    require(require_hash32(base_conductor.get("interface_binding_hash"), "interface_binding_hash") == binding_hash, "reviewed interface binding hash mismatch")
    admin = _validate_admin(value.get("admin"), request_nonce=request_nonce, conductor_id=conductor_id, observed_at=observed_at_unix_micros, base=base_conductor, runtime=runtime_conductor)
    app = _validate_app(value.get("app"), request_nonce=request_nonce, conductor_id=conductor_id, observed_at=observed_at_unix_micros, base=base_conductor, runtime=runtime_conductor)
    require(admin["endpoint_binding_hash"] != app["endpoint_binding_hash"], "admin and app interfaces may not share one endpoint binding")
    attestation_hash = require_hash32(value.get("attestation_hash"), "interface attestation_hash")
    expected_hash = sha256_b64(canonical_json_bytes({
        "domain": ATTESTATION_DOMAIN,
        "schema": INTERFACE_ATTESTATION_SCHEMA,
        "request_nonce": request_nonce,
        "observed_at_unix_micros": observed_at_unix_micros,
        "conductor_id": conductor_id,
        "interface_binding_hash": binding_hash,
        "admin_round_trip_hash": admin["round_trip_hash"],
        "app_round_trip_hash": app["round_trip_hash"],
    }))
    require(attestation_hash == expected_hash, "interface attestation hash mismatch")
    return {
        "schema": INTERFACE_ATTESTATION_SCHEMA,
        "request_nonce": request_nonce,
        "observed_at_unix_micros": observed_at_unix_micros,
        "conductor_id": conductor_id,
        "interface_binding_hash": binding_hash,
        "admin": admin,
        "app": app,
        "attestation_hash": attestation_hash,
    }
