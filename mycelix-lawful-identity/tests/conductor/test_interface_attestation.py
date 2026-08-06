#!/usr/bin/env python3
from __future__ import annotations

import base64
import hashlib
import json
import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from interface_attestation import (
    ADMIN_ROUND_TRIP_DOMAIN,
    APP_ROUND_TRIP_DOMAIN,
    ATTESTATION_DOMAIN,
    INTERFACE_ATTESTATION_SCHEMA,
    InterfaceAttestationError,
    canonical_json_bytes,
    interface_binding_hash,
    interface_binding_set_hash,
    sha256_b64,
    validate_interface_attestation,
)


def digest(label: str) -> str:
    return base64.b64encode(hashlib.sha256(label.encode()).digest()).decode("ascii")


def fixture() -> tuple[dict, dict, dict]:
    base = {
        "id": "verifier",
        "agent_pub_key": "agent-verifier",
        "database_identity_hash": digest("db"),
        "network_identity_hash": digest("network"),
        "admin_endpoint_binding_hash": digest("admin-endpoint"),
        "app_endpoint_binding_hash": digest("app-endpoint"),
        "app_signer_pub_key": "agent-verifier",
        "app_auth_token_hash": digest("app-token"),
    }
    base["interface_binding_hash"] = interface_binding_hash(base)
    runtime = {
        **base,
        "conductor_instance_id": digest("instance"),
        "cell_id_hash": digest("cell"),
    }
    nonce = digest("request-nonce")
    observed_at = 1_700_000_000_000_000
    admin = {
        "transport": "loopback-tcp",
        "endpoint_binding_hash": base["admin_endpoint_binding_hash"],
        "conductor_instance_id": runtime["conductor_instance_id"],
        "request_hash": digest("admin-request"),
        "response_hash": digest("admin-response"),
    }
    admin["round_trip_hash"] = sha256_b64(canonical_json_bytes({
        "domain": ADMIN_ROUND_TRIP_DOMAIN,
        "request_nonce": nonce,
        "conductor_id": base["id"],
        "observed_at_unix_micros": observed_at,
        "receipt": admin,
    }))
    app = {
        "transport": "loopback-tcp",
        "endpoint_binding_hash": base["app_endpoint_binding_hash"],
        "conductor_instance_id": runtime["conductor_instance_id"],
        "cell_id_hash": runtime["cell_id_hash"],
        "app_signer_pub_key": base["app_signer_pub_key"],
        "app_auth_token_hash": base["app_auth_token_hash"],
        "call_provenance_agent_pub_key": base["agent_pub_key"],
        "request_hash": digest("app-request"),
        "response_hash": digest("app-response"),
    }
    app["round_trip_hash"] = sha256_b64(canonical_json_bytes({
        "domain": APP_ROUND_TRIP_DOMAIN,
        "request_nonce": nonce,
        "conductor_id": base["id"],
        "observed_at_unix_micros": observed_at,
        "receipt": app,
    }))
    value = {
        "schema": INTERFACE_ATTESTATION_SCHEMA,
        "request_nonce": nonce,
        "observed_at_unix_micros": observed_at,
        "conductor_id": base["id"],
        "admin": admin,
        "app": app,
    }
    value["attestation_hash"] = sha256_b64(canonical_json_bytes({
        "domain": ATTESTATION_DOMAIN,
        "schema": INTERFACE_ATTESTATION_SCHEMA,
        "request_nonce": nonce,
        "observed_at_unix_micros": observed_at,
        "conductor_id": base["id"],
        "interface_binding_hash": base["interface_binding_hash"],
        "admin_round_trip_hash": admin["round_trip_hash"],
        "app_round_trip_hash": app["round_trip_hash"],
    }))
    return base, runtime, value


class InterfaceAttestationTests(unittest.TestCase):
    def test_valid_attestation(self) -> None:
        base, runtime, value = fixture()
        observed = validate_interface_attestation(
            value,
            request_nonce=value["request_nonce"],
            conductor_id=base["id"],
            observed_at_unix_micros=value["observed_at_unix_micros"],
            base_conductor=base,
            runtime_conductor=runtime,
        )
        self.assertEqual(observed["attestation_hash"], value["attestation_hash"])
        self.assertEqual(interface_binding_set_hash([base]), digest_from_set(base))

    def test_nonce_substitution_is_rejected(self) -> None:
        base, runtime, value = fixture()
        with self.assertRaises(InterfaceAttestationError):
            validate_interface_attestation(
                value,
                request_nonce=digest("other-nonce"),
                conductor_id=base["id"],
                observed_at_unix_micros=value["observed_at_unix_micros"],
                base_conductor=base,
                runtime_conductor=runtime,
            )

    def test_app_provenance_substitution_is_rejected(self) -> None:
        base, runtime, value = fixture()
        value = json.loads(json.dumps(value))
        value["app"]["call_provenance_agent_pub_key"] = "agent-attacker"
        with self.assertRaises(InterfaceAttestationError):
            validate_interface_attestation(
                value,
                request_nonce=value["request_nonce"],
                conductor_id=base["id"],
                observed_at_unix_micros=value["observed_at_unix_micros"],
                base_conductor=base,
                runtime_conductor=runtime,
            )

    def test_endpoint_substitution_is_rejected(self) -> None:
        base, runtime, value = fixture()
        value = json.loads(json.dumps(value))
        value["admin"]["endpoint_binding_hash"] = digest("other-endpoint")
        with self.assertRaises(InterfaceAttestationError):
            validate_interface_attestation(
                value,
                request_nonce=value["request_nonce"],
                conductor_id=base["id"],
                observed_at_unix_micros=value["observed_at_unix_micros"],
                base_conductor=base,
                runtime_conductor=runtime,
            )


def digest_from_set(base: dict) -> str:
    return sha256_b64(canonical_json_bytes({
        "domain": "MYCELIX:HolochainInterfaceBindingSet:v1",
        "conductors": [{"id": base["id"], "interface_binding_hash": base["interface_binding_hash"]}],
    }))


if __name__ == "__main__":
    unittest.main()
