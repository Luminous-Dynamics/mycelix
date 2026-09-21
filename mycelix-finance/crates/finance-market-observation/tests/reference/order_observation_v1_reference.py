#!/usr/bin/env python3
"""Independent FIN-MKT-002A V1 canonical observation oracle."""

import hashlib
import json
import pathlib
import struct

EXPECTED = {
    "event_identity": "bd0a7c994a58859f7fd99455433def8e9bfda2781149e90a23192042d60a8b19",
    "event_observation": "d772952075c747a3ca6d076218b093308aa2d8869a93479e0beeef098da17ae2",
    "fill_identity": "b6fb55af9d6f0ed1e4b2d5f696f9d8e8bf659efcd20c2ee7d807f492cd8dd134",
    "fill_observation": "d9cb8f1c8e6f4ea2dacb7cce3e0aeca060cb5f0df37e9fa6ba61d1c761202292",
    "adjust_identity": "f33bcce43385ce6d1f310439360baed8e3dbcc656890ce0e9ba286abbe2ba4d1",
    "adjust_observation": "1b6cec074f88d1767c6551a417d6b8b337ea0bf12db7aeedf5fc956869f5ce8c",
}

REVISION = 1


def u32(value):
    return struct.pack(">I", value)


def u64(value):
    return struct.pack(">Q", value)


def text(value):
    raw = value.encode("utf-8")
    return u32(len(raw)) + raw


def digest(value):
    return bytes.fromhex(value)


def profile(value):
    return text(value["profile_id"]) + u32(value["revision"]) + digest(value["digest"])


def subject_ref(value):
    return profile(value["subject_profile"]) + text(value["subject_id"])


def instrument_ref(value):
    return profile(value["instrument_profile"]) + text(value["instrument_id"])


def optional_text(value):
    if value is None:
        return b"\x00"
    return b"\x01" + text(value)


def observation_subject(value):
    return (
        digest(value["intent_commitment"])
        + subject_ref(value["account_subject"])
        + instrument_ref(value["instrument"])
        + profile(value["provider_profile"])
        + optional_text(value.get("provider_order_ref"))
    )


def observation_ref(value):
    return profile(value["observation_profile"]) + text(value["observation_id"])


def chronology(value):
    chronology_profile = value["chronology_profile"]
    sequence = value["sequence"]
    provider_time = value["provider_time"]
    if chronology_profile is None:
        if sequence is not None or provider_time is not None:
            raise ValueError("chronology evidence without chronology profile")
        return b"\x00"
    if sequence is None and provider_time is None:
        raise ValueError("chronology profile without chronology evidence")
    output = b"\x01" + profile(chronology_profile)
    output += b"\x00" if sequence is None else b"\x01" + u64(sequence)
    output += optional_text(provider_time)
    return output


EVENT_TAGS = {
    "SubmitAttemptObserved": 0,
    "ProviderAcceptedObserved": 1,
    "PendingNewObserved": 2,
    "WorkingObserved": 3,
    "CancelRequestedObserved": 4,
    "CancelAcceptedObserved": 5,
    "CancelRejectedObserved": 6,
    "ReplaceRequestedObserved": 7,
    "ReplaceAcceptedObserved": 8,
    "ReplaceRejectedObserved": 9,
    "DoneForDayObserved": 10,
    "ExpiredObserved": 11,
    "SuspendedOrHaltedObserved": 12,
    "ProviderRejectedObserved": 13,
    "SubmissionOutcomeUnknownObserved": 15,
}


def event_kind(value):
    if isinstance(value, str):
        return bytes([EVENT_TAGS[value]])
    return b"\x0e" + text(value["OpaqueProviderStatus"])


def quantity(value):
    return (
        profile(value["unit_profile"])
        + u64(value["amount"]["atomic_units"])
        + text(value["amount"]["asset"])
    )


def price(value):
    return (
        profile(value["pricing_profile"])
        + u64(value["quote_amount"]["atomic_units"])
        + text(value["quote_amount"]["asset"])
    )


def identity(domain, value):
    preimage = (
        domain
        + u32(REVISION)
        + observation_subject(value["subject"])
        + observation_ref(value["observation_ref"])
    )
    return hashlib.sha256(preimage).digest(), preimage


def main():
    fixture_path = pathlib.Path(__file__).parents[2] / "test-vectors" / "observations-v1.json"
    data = json.loads(fixture_path.read_text(encoding="utf-8"))

    event_id, event_id_preimage = identity(b"MYCELIX_FIN_MKT_EVENT_ID_V1\0", data["event"])
    event = data["event"]
    event_preimage = (
        b"MYCELIX_FIN_MKT_EVENT_OBS_V1\0"
        + u32(REVISION)
        + event_id
        + event_kind(event["event_kind"])
        + chronology(event["chronology"])
        + digest(event["source_evidence_commitment"])
    )

    fill_id, fill_id_preimage = identity(b"MYCELIX_FIN_MKT_FILL_ID_V1\0", data["fill"])
    fill = data["fill"]
    fill_preimage = (
        b"MYCELIX_FIN_MKT_FILL_OBS_V1\0"
        + u32(REVISION)
        + fill_id
        + text(fill["provider_execution_ref"])
        + quantity(fill["executed_quantity"])
        + price(fill["execution_price"])
        + optional_text(fill["venue_ref"])
        + chronology(fill["chronology"])
        + digest(fill["source_evidence_commitment"])
    )

    adjust_id, adjust_id_preimage = identity(
        b"MYCELIX_FIN_MKT_ADJUST_ID_V1\0", data["correction"]
    )
    correction = data["correction"]
    kind = correction["adjustment_kind"]
    if "Correction" in kind:
        item = kind["Correction"]
        kind_bytes = (
            b"\x00"
            + digest(item["prior_fill_commitment"])
            + digest(item["replacement_fill_commitment"])
        )
    else:
        kind_bytes = b"\x01" + digest(kind["Bust"]["prior_fill_commitment"])

    adjust_preimage = (
        b"MYCELIX_FIN_MKT_ADJUST_OBS_V1\0"
        + u32(REVISION)
        + adjust_id
        + kind_bytes
        + chronology(correction["chronology"])
        + digest(correction["source_evidence_commitment"])
    )

    actual = {
        "event_identity": event_id.hex(),
        "event_observation": hashlib.sha256(event_preimage).hexdigest(),
        "fill_identity": fill_id.hex(),
        "fill_observation": hashlib.sha256(fill_preimage).hexdigest(),
        "adjust_identity": adjust_id.hex(),
        "adjust_observation": hashlib.sha256(adjust_preimage).hexdigest(),
    }

    assert len(event_id_preimage) == 419
    assert len(event_preimage) == 203
    assert len(fill_id_preimage) == 422
    assert len(fill_preimage) == 441
    assert len(adjust_id_preimage) == 429
    assert len(adjust_preimage) == 268
    assert actual == EXPECTED, (actual, EXPECTED)

    for key, value in actual.items():
        print(f"{key}={value}")


if __name__ == "__main__":
    main()
