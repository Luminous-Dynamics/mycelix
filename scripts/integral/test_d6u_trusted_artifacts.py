#!/usr/bin/env python3
"""Unit tests for the pure-data portions of the D6U trusted verifier."""

import tempfile
from pathlib import Path

from verify_d6u_trusted_artifacts import load_record, verify_cases, verify_lock


def base_policy() -> dict:
    return {
        "cases": {
            "canonical-payload-accepted": {
                "outcome": "accepted",
                "zome_reached": True,
            }
        },
        "supplemental_substrate": {
            "future-expiry-rejection": "Future",
        },
        "application_check": {
            "id": "probe-local-d6s-commitment-mutation",
            "fragment": "result=d6s-commitment-mismatch;zome-reached=true",
        },
    }


def valid_log() -> str:
    return "
".join(
        [
            "D6U_CASE	canonical-payload-accepted	accepted	zome-reached=true	PASS",
            "D6U_RUNTIME_WITNESS	future-expiry-rejection	BadNonce("Future")",
            "D6U_SUBSTRATE_CHECK	future-expiry-rejection	BadNonce("Future")	PASS",
            "D6U_APPLICATION_CHECK	probe-local-d6s-commitment-mutation	result=d6s-commitment-mismatch;zome-reached=true	PASS",
        ]
    )


def test_valid_log_is_accepted() -> None:
    verify_cases(valid_log(), base_policy())


def test_case_tampering_is_rejected() -> None:
    tampered = valid_log().replace(
        "canonical-payload-accepted	accepted",
        "canonical-payload-accepted	semantic-rejected",
    )
    try:
        verify_cases(tampered, base_policy())
    except AssertionError:
        return
    raise AssertionError("tampered D6U case was accepted")


def test_duplicate_case_is_rejected() -> None:
    tampered = valid_log() + "
" + (
        "D6U_CASE	canonical-payload-accepted	accepted	zome-reached=true	PASS"
    )
    try:
        verify_cases(tampered, base_policy())
    except AssertionError:
        return
    raise AssertionError("duplicate D6U case was accepted")


def test_lock_provenance_is_rejected_when_tampered() -> None:
    package = {
        "name": "holochain",
        "version": "0.7.0",
        "source": "registry+https://github.com/rust-lang/crates.io-index",
        "checksum": "a" * 64,
    }
    policy = {
        "lock_packages": {"holochain": "0.7.0"},
        "lock_source": "registry+https://github.com/rust-lang/crates.io-index",
    }

    with tempfile.TemporaryDirectory() as tmp:
        lock = Path(tmp) / "Cargo.lock"
        lock.write_text(
            'version = 3\n\n[[package]]\n'
            + "\n".join(f"{key} = "{value}"" for key, value in package.items())
            + "\n",
            encoding="utf-8",
        )
        verify_lock(lock, policy)

        lock.write_text(
            lock.read_text(encoding="utf-8").replace("checksum = " + '"' + ("a" * 64) + '"', ""),
            encoding="utf-8",
        )
        try:
            verify_lock(lock, policy)
        except AssertionError:
            return
        raise AssertionError("malformed lock checksum was accepted")


def test_duplicate_record_key_is_rejected() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        record = Path(tmp) / "record.txt"
        record.write_text(
            "D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE
"
            "status=runtime-reference-evidence
"
            "status=runtime-reference-evidence
",
            encoding="utf-8",
        )
        try:
            load_record(record)
        except AssertionError:
            return
        raise AssertionError("duplicate record field was accepted")


if __name__ == "__main__":
    tests = [
        test_valid_log_is_accepted,
        test_case_tampering_is_rejected,
        test_duplicate_case_is_rejected,
        test_lock_provenance_is_rejected_when_tampered,
        test_duplicate_record_key_is_rejected,
    ]
    for test in tests:
        test()
    print(f"verified D6U trusted verifier self-tests: {len(tests)}/{len(tests)}")
