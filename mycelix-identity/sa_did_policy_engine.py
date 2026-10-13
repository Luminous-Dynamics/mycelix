#!/usr/bin/env python3
"""Synthetic-only SA-DID policy oracle.

This module evaluates synthetic policy signals, not credentials. It does not
parse credential formats, verify signatures, retrieve issuer keys, query DHA,
or establish legal identity. Its only positive result is a test-corpus outcome
when a vector is explicitly synthetic and production-ineligible.
"""
from __future__ import annotations

from typing import Any

POLICY_INPUT_FIELDS = {
    "user_approved",
    "alternative_route_required",
    "source_state",
    "profile_supported",
    "issuer_trusted",
    "proof_valid",
    "schema_valid",
    "audience_matches",
    "nonce_fresh",
    "holder_binding_valid",
    "time_state",
    "credential_status",
    "privacy_safe",
    "authorization_scope_valid",
}
BOOLEAN_FIELDS = {
    "user_approved",
    "alternative_route_required",
    "profile_supported",
    "issuer_trusted",
    "proof_valid",
    "schema_valid",
    "audience_matches",
    "nonce_fresh",
    "holder_binding_valid",
    "privacy_safe",
    "authorization_scope_valid",
}
ENUM_FIELDS = {
    "source_state": {"available", "unavailable"},
    "time_state": {"valid", "expired", "unknown"},
    "credential_status": {"valid", "revoked", "unknown", "unavailable", "stale"},
}
DISPOSITIONS = {"verified", "rejected", "indeterminate"}


def _decision(disposition: str, reason: str) -> tuple[str, str]:
    """Return the policy disposition and stable reason code."""
    return disposition, reason


def evaluate_synthetic_vector(vector: Any) -> tuple[str, str]:
    """Evaluate a frozen synthetic policy vector; never a real credential.

    Raises ValueError for malformed or production-eligible fixtures. Callers
    must not turn exceptions into a VERIFIED result.
    """
    if not isinstance(vector, dict):
        raise ValueError("vector must be a JSON-like object")
    if vector.get("synthetic_fixture") is not True:
        raise ValueError("only explicitly synthetic fixtures are supported")
    if vector.get("production_eligible") is not False:
        raise ValueError("synthetic policy oracle refuses production-eligible vectors")

    anchor_ref = vector.get("trust_anchor_ref")
    if not isinstance(anchor_ref, str) or not (
        anchor_ref == "none" or anchor_ref.startswith("test-only:")
    ):
        raise ValueError("trust_anchor_ref must be none or an explicit test-only reference")

    evidence = vector.get("policy_inputs")
    if not isinstance(evidence, dict) or set(evidence) != POLICY_INPUT_FIELDS:
        raise ValueError("policy_inputs must match the closed synthetic input schema")

    for field in BOOLEAN_FIELDS:
        if type(evidence.get(field)) is not bool:
            raise ValueError(f"policy_inputs.{field} must be a Boolean")

    for field, allowed_values in ENUM_FIELDS.items():
        value = evidence.get(field)
        if not isinstance(value, str) or value not in allowed_values:
            raise ValueError(f"policy_inputs.{field} must be one of {sorted(allowed_values)}")

    if evidence["issuer_trusted"] and not anchor_ref.startswith("test-only:"):
        raise ValueError("a trusted synthetic issuer must use a test-only trust reference")

    # Preserve alternative access before attempting identity checks.
    if evidence["alternative_route_required"]:
        return _decision("indeterminate", "alternative_route_required")
    if not evidence["user_approved"]:
        return _decision("indeterminate", "user_declined")
    if evidence["source_state"] == "unavailable":
        return _decision("indeterminate", "source_unavailable")
    if not evidence["profile_supported"]:
        return _decision("indeterminate", "unsupported_profile")
    if not evidence["issuer_trusted"]:
        return _decision("rejected", "issuer_not_trusted")
    if not evidence["proof_valid"]:
        return _decision("rejected", "invalid_proof")
    if not evidence["schema_valid"]:
        return _decision("rejected", "schema_or_profile_mismatch")
    if not evidence["audience_matches"]:
        return _decision("rejected", "wrong_audience")
    if not evidence["nonce_fresh"]:
        return _decision("rejected", "nonce_replay")
    if not evidence["holder_binding_valid"]:
        return _decision("rejected", "holder_binding_failed")
    if evidence["time_state"] == "expired":
        return _decision("rejected", "credential_expired")
    if evidence["time_state"] == "unknown":
        return _decision("indeterminate", "credential_time_unknown")

    status_outcomes = {
        "revoked": ("rejected", "credential_revoked"),
        "unknown": ("indeterminate", "status_unknown"),
        "unavailable": ("indeterminate", "status_unavailable"),
        "stale": ("indeterminate", "status_stale"),
    }
    if evidence["credential_status"] in status_outcomes:
        return status_outcomes[evidence["credential_status"]]

    if not evidence["privacy_safe"]:
        return _decision("rejected", "privacy_boundary_violation")
    if not evidence["authorization_scope_valid"]:
        return _decision("rejected", "authorization_scope_violation")

    # "verified" here means only that the synthetic policy inputs satisfy
    # this test oracle. It is not a cryptographic or real-world identity result.
    return _decision("verified", "none")


__all__ = ["evaluate_synthetic_vector", "POLICY_INPUT_FIELDS"]
