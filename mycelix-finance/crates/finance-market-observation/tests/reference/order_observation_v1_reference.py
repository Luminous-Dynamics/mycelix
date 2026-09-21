#!/usr/bin/env python3
"""Independent FIN-MKT-002A V1 identity/semantic/evidence oracle."""

import hashlib
import json
import pathlib
import struct

REVISION = 1

EXPECTED = {
    "event_identity": "bd0a7c994a58859f7fd99455433def8e9bfda2781149e90a23192042d60a8b19",
    "event_semantic": "e1cb0ee74f32f092ce23eae260196d7edff1f4db6f65619a65ad3df5bd740d57",
    "event_evidence": "dc2a1ad70de2b9219da30e5cd9af8cd78473008ab7eca83204ec9b6d3363345c",
    "fill_identity": "b6fb55af9d6f0ed1e4b2d5f696f9d8e8bf659efcd20c2ee7d807f492cd8dd134",
    "fill_semantic": "23121d3de29ccc6f5dce122a7b7475c6ebd86670a063e5ea54ff9901e82ee196",
    "fill_evidence": "19816667fa931cee5b9ecd9a8579d8d39facc3927c05719dbb736c4f3cad71d0",
    "adjust_identity": "f33bcce43385ce6d1f310439360baed8e3dbcc656890ce0e9ba286abbe2ba4d1",
    "adjust_semantic": "62e2abcd01078dba27daed66d69e0a1fa9d9a43b6333749f051e5f433d792e73",
    "adjust_evidence": "fcec2b73c14fbd78eaf4e18273fdeb2ada4655fe7bed5f337cb163e3b976c476",
}

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
    return b"\x00" if value is None else b"\x01" + text(value)


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
    has_profile = chronology_profile is not None
    has_evidence = sequence is not None or provider_time is not None
    if has_profile != has_evidence:
        raise ValueError("chronology profile and evidence must be present together")
    if not has_profile:
        return b"\x00"
    result = b"\x01" + profile(chronology_profile)
    result += b"\x00" if sequence is None else b"\x01" + u64(sequence)
    result += optional_text(provider_time)
    return result


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


def common_identity(domain, value):
    preimage = (
        domain
        + u32(REVISION)
        + observation_subject(value["subject"])
        + observation_ref(value["observation_ref"])
    )
    return preimage, hashlib.sha256(preimage).digest()


def evidence_preimage(domain, identity, semantic, source_evidence):
    return domain + u32(REVISION) + identity + semantic + digest(source_evidence)


def main():
    fixture = pathlib.Path(__file__).parents[2] / "test-vectors" / "observations-v1.json"
    data = json.loads(fixture.read_text(encoding="utf-8"))

    event = data["event"]
    event_id_pre, event_id = common_identity(b"MYCELIX_FIN_MKT_EVENT_ID_V1\0", event)
    event_sem_pre = (
        b"MYCELIX_FIN_MKT_EVENT_SEM_V1\0"
        + u32(REVISION)
        + event_id
        + event_kind(event["event_kind"])
        + chronology(event["chronology"])
    )
    event_sem = hashlib.sha256(event_sem_pre).digest()
    event_evidence_pre = evidence_preimage(
        b"MYCELIX_FIN_MKT_EVENT_EVIDENCE_V1\0",
        event_id,
        event_sem,
        event["source_evidence_commitment"],
    )

    fill = data["fill"]
    fill_id_pre, fill_id = common_identity(b"MYCELIX_FIN_MKT_FILL_ID_V1\0", fill)
    fill_sem_pre = (
        b"MYCELIX_FIN_MKT_FILL_SEM_V1\0"
        + u32(REVISION)
        + fill_id
        + text(fill["provider_execution_ref"])
        + quantity(fill["executed_quantity"])
        + price(fill["execution_price"])
        + optional_text(fill["venue_ref"])
        + chronology(fill["chronology"])
    )
    fill_sem = hashlib.sha256(fill_sem_pre).digest()
    fill_evidence_pre = evidence_preimage(
        b"MYCELIX_FIN_MKT_FILL_EVIDENCE_V1\0",
        fill_id,
        fill_sem,
        fill["source_evidence_commitment"],
    )

    adjust = data["correction"]
    adjust_id_pre, adjust_id = common_identity(b"MYCELIX_FIN_MKT_ADJUST_ID_V1\0", adjust)
    kind = adjust["adjustment_kind"]
    if "Correction" in kind:
        correction = kind["Correction"]
        adjust_kind = (
            b"\x00"
            + digest(correction["prior_fill_commitment"])
            + digest(correction["replacement_fill_commitment"])
        )
    else:
        adjust_kind = b"\x01" + digest(kind["Bust"]["prior_fill_commitment"])
    adjust_sem_pre = (
        b"MYCELIX_FIN_MKT_ADJUST_SEM_V1\0"
        + u32(REVISION)
        + adjust_id
        + adjust_kind
        + chronology(adjust["chronology"])
    )
    adjust_sem = hashlib.sha256(adjust_sem_pre).digest()
    adjust_evidence_pre = evidence_preimage(
        b"MYCELIX_FIN_MKT_ADJUST_EVIDENCE_V1\0",
        adjust_id,
        adjust_sem,
        adjust["source_evidence_commitment"],
    )

    preimages = {
        "event_identity": event_id_pre,
        "event_semantic": event_sem_pre,
        "event_evidence": event_evidence_pre,
        "fill_identity": fill_id_pre,
        "fill_semantic": fill_sem_pre,
        "fill_evidence": fill_evidence_pre,
        "adjust_identity": adjust_id_pre,
        "adjust_semantic": adjust_sem_pre,
        "adjust_evidence": adjust_evidence_pre,
    }
    expected_lengths = {
        "event_identity": 419,
        "event_semantic": 171,
        "event_evidence": 134,
        "fill_identity": 422,
        "fill_semantic": 409,
        "fill_evidence": 133,
        "adjust_identity": 429,
        "adjust_semantic": 236,
        "adjust_evidence": 135,
    }

    actual = {key: hashlib.sha256(value).hexdigest() for key, value in preimages.items()}
    for key, length in expected_lengths.items():
        assert len(preimages[key]) == length, (key, len(preimages[key]), length)
    assert actual == EXPECTED, (actual, EXPECTED)

    for key in EXPECTED:
        print(f"{key}={actual[key]}")


if __name__ == "__main__":
    main()
