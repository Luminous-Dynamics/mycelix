#!/usr/bin/env python3
"""SUP-CIV-000D1A protected-envelope semantic reference model.

This is deliberately NOT a cryptographic implementation. All cryptographic
objects are opaque 32-byte commitments represented as lowercase hex. The
SHA-256 digest used by this oracle is a fixture-only reference-model linkage
digest and is not a normative runtime envelope ID.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Iterable, Optional, Tuple

SCHEMA = "protected-envelope-semantic-v1"
RETIREMENT_SCHEMA = "protected-envelope-retirement-v1"
DEK_GENERATION_CLASS = "random-per-payload-dek-v1"
DIGEST_AUTHORITY = "fixture-only-not-runtime-envelope-id"
_HEX64 = re.compile(r"^[0-9a-f]{64}$")


class Refusal(ValueError):
    pass


def fixture_ref(label: str) -> str:
    return hashlib.sha256(("fixture:" + label).encode("utf-8")).hexdigest()


@dataclass(frozen=True, order=True)
class Recipient:
    subject_ref: str
    key_id: str
    key_profile_ref: str
    wrap_profile_ref: str
    wrapped_dek_commitment: str
    wrap_context_commitment: str


@dataclass(frozen=True)
class Envelope:
    schema: str
    payload_subject_ref: str
    payload_version: int
    envelope_revision: int
    key_epoch: int
    aead_profile_ref: str
    ciphertext_commitment: str
    aad_commitment: str
    dek_generation_class: str
    dek_generation_receipt_ref: str
    recipients: Tuple[Recipient, ...]
    previous_envelope_ref: Optional[str]
    creation_profile_ref: str


@dataclass(frozen=True)
class RetirementReceipt:
    schema: str
    envelope_ref: str
    disposition_profile_ref: str
    evidence_ref: str


def canonical_recipients(recipients: Iterable[Recipient]) -> Tuple[Recipient, ...]:
    return tuple(sorted(recipients, key=lambda r: (r.subject_ref, r.key_id)))


def _check_ref(value: object, name: str) -> None:
    if not isinstance(value, str) or _HEX64.fullmatch(value) is None:
        raise Refusal(f"{name}:invalid-ref")


def validate_envelope(envelope: Envelope, *, create: bool = False) -> None:
    if envelope.schema != SCHEMA:
        raise Refusal("schema")
    for name in (
        "payload_subject_ref",
        "aead_profile_ref",
        "ciphertext_commitment",
        "aad_commitment",
        "dek_generation_receipt_ref",
        "creation_profile_ref",
    ):
        _check_ref(getattr(envelope, name), name)
    if envelope.previous_envelope_ref is not None:
        _check_ref(envelope.previous_envelope_ref, "previous_envelope_ref")
    if min(envelope.payload_version, envelope.envelope_revision, envelope.key_epoch) < 1:
        raise Refusal("nonpositive-counter")
    if envelope.dek_generation_class != DEK_GENERATION_CLASS:
        raise Refusal("dek-generation-class")
    if not envelope.recipients:
        raise Refusal("empty-recipient-set")

    subjects = set()
    keys = set()
    for recipient in envelope.recipients:
        for name in (
            "subject_ref",
            "key_id",
            "key_profile_ref",
            "wrap_profile_ref",
            "wrapped_dek_commitment",
            "wrap_context_commitment",
        ):
            _check_ref(getattr(recipient, name), f"recipient.{name}")
        if recipient.subject_ref in subjects:
            raise Refusal("duplicate-recipient-subject")
        if recipient.key_id in keys:
            raise Refusal("duplicate-recipient-key")
        subjects.add(recipient.subject_ref)
        keys.add(recipient.key_id)

    if envelope.recipients != canonical_recipients(envelope.recipients):
        raise Refusal("recipient-order-noncanonical")

    if create:
        if envelope.previous_envelope_ref is not None:
            raise Refusal("create-has-previous")
        if (envelope.payload_version, envelope.envelope_revision, envelope.key_epoch) != (1, 1, 1):
            raise Refusal("create-counters")


def _envelope_dict(envelope: Envelope) -> dict:
    result = asdict(envelope)
    result["recipients"] = [asdict(r) for r in envelope.recipients]
    return result


def reference_model_digest(envelope: Envelope) -> str:
    """Fixture-only lineage digest; NOT a normative protocol/envelope ID."""
    payload = json.dumps(
        _envelope_dict(envelope),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(b"sup-civ-000d1a-reference-only\x00" + payload).hexdigest()


def _by_subject(envelope: Envelope) -> dict[str, Recipient]:
    return {recipient.subject_ref: recipient for recipient in envelope.recipients}


def _check_common_transition(base: Envelope, candidate: Envelope) -> None:
    validate_envelope(base)
    validate_envelope(candidate)
    for attribute in ("schema", "payload_subject_ref", "creation_profile_ref", "aead_profile_ref"):
        if getattr(candidate, attribute) != getattr(base, attribute):
            raise Refusal(f"{attribute}-change")
    if candidate.envelope_revision != base.envelope_revision + 1:
        raise Refusal("revision-not-next")
    if candidate.previous_envelope_ref != reference_model_digest(base):
        raise Refusal("previous-ref-mismatch")


def _require_new_epoch_and_ciphertext(base: Envelope, candidate: Envelope, prefix: str) -> None:
    if candidate.key_epoch != base.key_epoch + 1:
        raise Refusal(f"{prefix}-key-epoch")
    if candidate.ciphertext_commitment == base.ciphertext_commitment:
        raise Refusal(f"{prefix}-ciphertext-not-rotated")
    if candidate.aad_commitment == base.aad_commitment:
        raise Refusal(f"{prefix}-aad-not-rotated")
    if candidate.dek_generation_receipt_ref == base.dek_generation_receipt_ref:
        raise Refusal(f"{prefix}-new-dek-required")


def _require_all_remaining_wraps_rotated(base: Envelope, candidate: Envelope, prefix: str) -> None:
    old = _by_subject(base)
    for subject, recipient in _by_subject(candidate).items():
        if subject in old and recipient.wrapped_dek_commitment == old[subject].wrapped_dek_commitment:
            raise Refusal(f"{prefix}-wrap-not-rotated")


def validate_transition(
    base: Envelope,
    candidate: Envelope,
    operation: str,
    *,
    retired_refs: frozenset[str] = frozenset(),
) -> None:
    if reference_model_digest(base) in retired_refs:
        raise Refusal("base-retired")

    _check_common_transition(base, candidate)
    old = _by_subject(base)
    new = _by_subject(candidate)
    old_subjects = set(old)
    new_subjects = set(new)

    if operation == "add-recipient-current-epoch":
        if (candidate.payload_version, candidate.key_epoch) != (base.payload_version, base.key_epoch):
            raise Refusal("add-current-counter-change")
        if (candidate.ciphertext_commitment, candidate.aad_commitment) != (
            base.ciphertext_commitment,
            base.aad_commitment,
        ):
            raise Refusal("add-current-payload-change")
        if candidate.dek_generation_receipt_ref != base.dek_generation_receipt_ref:
            raise Refusal("add-current-dek-change")
        if len(new_subjects - old_subjects) != 1 or old_subjects - new_subjects:
            raise Refusal("add-current-recipient-set")
        for subject in old_subjects:
            if new[subject] != old[subject]:
                raise Refusal("add-current-mutated-existing-recipient")
        return

    if operation == "add-recipient-future-only":
        if candidate.payload_version != base.payload_version:
            raise Refusal("add-future-payload-version-change")
        _require_new_epoch_and_ciphertext(base, candidate, "add-future")
        if len(new_subjects - old_subjects) != 1 or old_subjects - new_subjects:
            raise Refusal("add-future-recipient-set")
        _require_all_remaining_wraps_rotated(base, candidate, "add-future")
        return

    if operation == "remove-recipient-forward-exclusion":
        if candidate.payload_version != base.payload_version:
            raise Refusal("remove-payload-version-change")
        _require_new_epoch_and_ciphertext(base, candidate, "remove")
        if len(old_subjects - new_subjects) != 1 or new_subjects - old_subjects:
            raise Refusal("remove-recipient-set")
        _require_all_remaining_wraps_rotated(base, candidate, "remove")
        return

    if operation == "rotate-after-compromise":
        if candidate.payload_version != base.payload_version:
            raise Refusal("rotate-payload-version-change")
        _require_new_epoch_and_ciphertext(base, candidate, "rotate")
        if new_subjects != old_subjects:
            raise Refusal("rotate-recipient-set-change")
        _require_all_remaining_wraps_rotated(base, candidate, "rotate")
        return

    if operation == "rewrap-recipient-key":
        if (candidate.payload_version, candidate.key_epoch) != (base.payload_version, base.key_epoch):
            raise Refusal("rewrap-counter-change")
        if (candidate.ciphertext_commitment, candidate.aad_commitment) != (
            base.ciphertext_commitment,
            base.aad_commitment,
        ):
            raise Refusal("rewrap-payload-change")
        if candidate.dek_generation_receipt_ref != base.dek_generation_receipt_ref:
            raise Refusal("rewrap-dek-change")
        if new_subjects != old_subjects:
            raise Refusal("rewrap-recipient-set")
        changed = [subject for subject in old_subjects if new[subject] != old[subject]]
        if len(changed) != 1:
            raise Refusal("rewrap-exactly-one")
        subject = changed[0]
        if new[subject].key_id == old[subject].key_id:
            raise Refusal("rewrap-key-id-not-changed")
        if new[subject].wrapped_dek_commitment == old[subject].wrapped_dek_commitment:
            raise Refusal("rewrap-wrap-not-changed")
        if new[subject].wrap_context_commitment == old[subject].wrap_context_commitment:
            raise Refusal("rewrap-context-not-changed")
        return

    if operation == "replace-payload":
        if candidate.payload_version != base.payload_version + 1:
            raise Refusal("payload-version-not-next")
        _require_new_epoch_and_ciphertext(base, candidate, "payload")
        if new_subjects != old_subjects:
            raise Refusal("payload-recipient-set-change")
        _require_all_remaining_wraps_rotated(base, candidate, "payload")
        return

    raise Refusal("unknown-operation")


def validate_retirement(base: Envelope, receipt: RetirementReceipt) -> None:
    validate_envelope(base)
    if receipt.schema != RETIREMENT_SCHEMA:
        raise Refusal("retirement-schema")
    for name in ("envelope_ref", "disposition_profile_ref", "evidence_ref"):
        _check_ref(getattr(receipt, name), name)
    if receipt.envelope_ref != reference_model_digest(base):
        raise Refusal("retirement-envelope-mismatch")


def _recipient(name: str, *, key: Optional[str] = None, wrap: Optional[str] = None, context: Optional[str] = None) -> Recipient:
    return Recipient(
        fixture_ref(f"subject:{name}"),
        fixture_ref(f"key:{key or name}"),
        fixture_ref(f"key-profile:{key or name}"),
        fixture_ref("wrap-profile"),
        fixture_ref(f"wrapped:{wrap or name}"),
        fixture_ref(f"wrap-context:{context or name}"),
    )


def _candidate(
    base: Envelope,
    *,
    payload_version: Optional[int] = None,
    envelope_revision: Optional[int] = None,
    key_epoch: Optional[int] = None,
    ciphertext: Optional[str] = None,
    aad: Optional[str] = None,
    dek_receipt: Optional[str] = None,
    recipients: Optional[Iterable[Recipient]] = None,
    previous: Optional[str] = None,
) -> Envelope:
    return replace(
        base,
        payload_version=base.payload_version if payload_version is None else payload_version,
        envelope_revision=(base.envelope_revision + 1 if envelope_revision is None else envelope_revision),
        key_epoch=base.key_epoch if key_epoch is None else key_epoch,
        ciphertext_commitment=base.ciphertext_commitment if ciphertext is None else ciphertext,
        aad_commitment=base.aad_commitment if aad is None else aad,
        dek_generation_receipt_ref=(
            base.dek_generation_receipt_ref if dek_receipt is None else dek_receipt
        ),
        recipients=(
            base.recipients if recipients is None else canonical_recipients(tuple(recipients))
        ),
        previous_envelope_ref=reference_model_digest(base) if previous is None else previous,
    )


def _observe(callable_) -> tuple[str, Optional[str]]:
    try:
        callable_()
        return ("Admit", None)
    except Refusal as exc:
        return ("Refuse", str(exc))


def build_cases() -> dict[str, tuple[str, Optional[str]]]:
    a = _recipient("A")
    b = _recipient("B")
    c = _recipient("C")
    base = Envelope(
        SCHEMA,
        fixture_ref("payload"),
        1,
        1,
        1,
        fixture_ref("aead-profile"),
        fixture_ref("ciphertext:1"),
        fixture_ref("aad:1"),
        DEK_GENERATION_CLASS,
        fixture_ref("dek-generation:1"),
        canonical_recipients((a, b)),
        None,
        fixture_ref("creation-profile"),
    )

    cases: dict[str, tuple[str, Optional[str]]] = {}
    cases["valid_create"] = _observe(lambda: validate_envelope(base, create=True))
    cases["create_session_derived_dek"] = _observe(
        lambda: validate_envelope(replace(base, dek_generation_class="session-derived"), create=True)
    )
    cases["create_duplicate_subject"] = _observe(
        lambda: validate_envelope(
            replace(base, recipients=canonical_recipients((a, replace(a, key_id=fixture_ref("key:other"))))),
            create=True,
        )
    )
    cases["create_duplicate_key"] = _observe(
        lambda: validate_envelope(
            replace(base, recipients=canonical_recipients((a, replace(b, key_id=a.key_id)))),
            create=True,
        )
    )
    cases["create_noncanonical_recipient_order"] = _observe(
        lambda: validate_envelope(replace(base, recipients=tuple(reversed(base.recipients))), create=True)
    )
    cases["create_bad_counters"] = _observe(
        lambda: validate_envelope(replace(base, key_epoch=2), create=True)
    )

    add_current = _candidate(base, recipients=(a, b, c))
    cases["valid_add_recipient_current_epoch"] = _observe(
        lambda: validate_transition(base, add_current, "add-recipient-current-epoch")
    )
    cases["add_current_mutates_existing_recipient"] = _observe(
        lambda: validate_transition(
            base,
            replace(
                add_current,
                recipients=canonical_recipients(
                    (replace(a, wrapped_dek_commitment=fixture_ref("wrapped:A:mutated")), b, c)
                ),
            ),
            "add-recipient-current-epoch",
        )
    )
    cases["add_current_changes_dek"] = _observe(
        lambda: validate_transition(
            base,
            replace(add_current, dek_generation_receipt_ref=fixture_ref("dek-generation:2")),
            "add-recipient-current-epoch",
        )
    )

    a2 = replace(a, wrapped_dek_commitment=fixture_ref("wrapped:A:epoch2"))
    b2 = replace(b, wrapped_dek_commitment=fixture_ref("wrapped:B:epoch2"))
    add_future = _candidate(
        base,
        key_epoch=2,
        ciphertext=fixture_ref("ciphertext:add-future"),
        aad=fixture_ref("aad:add-future"),
        dek_receipt=fixture_ref("dek-generation:add-future"),
        recipients=(a2, b2, c),
    )
    cases["valid_add_recipient_future_only"] = _observe(
        lambda: validate_transition(base, add_future, "add-recipient-future-only")
    )
    cases["add_future_same_epoch"] = _observe(
        lambda: validate_transition(
            base, replace(add_future, key_epoch=1), "add-recipient-future-only"
        )
    )
    cases["add_future_stale_existing_wrap"] = _observe(
        lambda: validate_transition(
            base,
            replace(add_future, recipients=canonical_recipients((a, b2, c))),
            "add-recipient-future-only",
        )
    )

    remove = _candidate(
        base,
        key_epoch=2,
        ciphertext=fixture_ref("ciphertext:remove"),
        aad=fixture_ref("aad:remove"),
        dek_receipt=fixture_ref("dek-generation:remove"),
        recipients=(a2,),
    )
    cases["valid_remove_recipient_forward_exclusion"] = _observe(
        lambda: validate_transition(base, remove, "remove-recipient-forward-exclusion")
    )
    cases["remove_same_epoch"] = _observe(
        lambda: validate_transition(
            base, replace(remove, key_epoch=1), "remove-recipient-forward-exclusion"
        )
    )
    cases["remove_same_ciphertext"] = _observe(
        lambda: validate_transition(
            base,
            replace(remove, ciphertext_commitment=base.ciphertext_commitment),
            "remove-recipient-forward-exclusion",
        )
    )
    cases["remove_stale_remaining_wrap"] = _observe(
        lambda: validate_transition(
            base,
            replace(remove, recipients=(a,)),
            "remove-recipient-forward-exclusion",
        )
    )
    single = replace(base, recipients=(a,))
    remove_last = _candidate(
        single,
        key_epoch=2,
        ciphertext=fixture_ref("ciphertext:remove-last"),
        aad=fixture_ref("aad:remove-last"),
        dek_receipt=fixture_ref("dek-generation:remove-last"),
        recipients=(),
    )
    cases["remove_last_recipient"] = _observe(
        lambda: validate_transition(single, remove_last, "remove-recipient-forward-exclusion")
    )

    a3 = replace(a, wrapped_dek_commitment=fixture_ref("wrapped:A:compromise"))
    b3 = replace(b, wrapped_dek_commitment=fixture_ref("wrapped:B:compromise"))
    rotate = _candidate(
        base,
        key_epoch=2,
        ciphertext=fixture_ref("ciphertext:compromise"),
        aad=fixture_ref("aad:compromise"),
        dek_receipt=fixture_ref("dek-generation:compromise"),
        recipients=(a3, b3),
    )
    cases["valid_rotate_after_compromise"] = _observe(
        lambda: validate_transition(base, rotate, "rotate-after-compromise")
    )
    cases["rotate_changes_recipient_set"] = _observe(
        lambda: validate_transition(
            base,
            replace(rotate, recipients=(a3,)),
            "rotate-after-compromise",
        )
    )

    a_rewrapped = replace(
        a,
        key_id=fixture_ref("key:A:replacement"),
        key_profile_ref=fixture_ref("key-profile:A:replacement"),
        wrapped_dek_commitment=fixture_ref("wrapped:A:replacement"),
        wrap_context_commitment=fixture_ref("wrap-context:A:replacement"),
    )
    rewrap = _candidate(base, recipients=(a_rewrapped, b))
    cases["valid_rewrap_recipient_key"] = _observe(
        lambda: validate_transition(base, rewrap, "rewrap-recipient-key")
    )
    cases["rewrap_same_key"] = _observe(
        lambda: validate_transition(
            base,
            replace(
                rewrap,
                recipients=canonical_recipients((replace(a_rewrapped, key_id=a.key_id), b)),
            ),
            "rewrap-recipient-key",
        )
    )
    cases["rewrap_changes_dek"] = _observe(
        lambda: validate_transition(
            base,
            replace(rewrap, dek_generation_receipt_ref=fixture_ref("dek-generation:rewrap")),
            "rewrap-recipient-key",
        )
    )

    a4 = replace(a, wrapped_dek_commitment=fixture_ref("wrapped:A:payload2"))
    b4 = replace(b, wrapped_dek_commitment=fixture_ref("wrapped:B:payload2"))
    payload2 = _candidate(
        base,
        payload_version=2,
        key_epoch=2,
        ciphertext=fixture_ref("ciphertext:payload2"),
        aad=fixture_ref("aad:payload2"),
        dek_receipt=fixture_ref("dek-generation:payload2"),
        recipients=(a4, b4),
    )
    cases["valid_replace_payload"] = _observe(
        lambda: validate_transition(base, payload2, "replace-payload")
    )
    cases["replace_skips_payload_version"] = _observe(
        lambda: validate_transition(
            base, replace(payload2, payload_version=3), "replace-payload"
        )
    )

    cases["wrong_previous_ref"] = _observe(
        lambda: validate_transition(
            base,
            replace(add_current, previous_envelope_ref=fixture_ref("wrong-previous")),
            "add-recipient-current-epoch",
        )
    )
    cases["revision_skip"] = _observe(
        lambda: validate_transition(
            base,
            replace(add_current, envelope_revision=3),
            "add-recipient-current-epoch",
        )
    )

    retirement = RetirementReceipt(
        RETIREMENT_SCHEMA,
        reference_model_digest(base),
        fixture_ref("retirement-profile"),
        fixture_ref("retirement-evidence"),
    )
    cases["valid_retirement"] = _observe(lambda: validate_retirement(base, retirement))
    cases["retirement_wrong_envelope"] = _observe(
        lambda: validate_retirement(
            base, replace(retirement, envelope_ref=fixture_ref("wrong-envelope"))
        )
    )
    cases["transition_after_retirement"] = _observe(
        lambda: validate_transition(
            base,
            add_current,
            "add-recipient-current-epoch",
            retired_refs=frozenset((reference_model_digest(base),)),
        )
    )
    cases["unknown_operation"] = _observe(
        lambda: validate_transition(base, add_current, "set-state")
    )
    return cases


def validate_manifest(manifest: dict, observed: dict[str, tuple[str, Optional[str]]]) -> None:
    if manifest.get("schema") != "sup-civ-000d1a-reference-manifest-v1":
        raise SystemExit("manifest schema mismatch")
    if manifest.get("authority") != "semantic-reference-model-only":
        raise SystemExit("manifest authority mismatch")
    expected = manifest.get("cases")
    if not isinstance(expected, list):
        raise SystemExit("manifest cases missing")
    expected_map = {}
    for row in expected:
        if set(row) != {"id", "disposition", "reason"}:
            raise SystemExit(f"bad manifest row: {row!r}")
        if row["id"] in expected_map:
            raise SystemExit(f"duplicate manifest case: {row['id']}")
        expected_map[row["id"]] = (row["disposition"], row["reason"])

    if set(expected_map) != set(observed):
        missing = sorted(set(expected_map) - set(observed))
        extra = sorted(set(observed) - set(expected_map))
        raise SystemExit(f"case-set mismatch missing={missing} extra={extra}")

    mismatches = {
        case_id: {"expected": expected_map[case_id], "observed": observed[case_id]}
        for case_id in sorted(observed)
        if expected_map[case_id] != observed[case_id]
    }
    if mismatches:
        raise SystemExit("case mismatch: " + json.dumps(mismatches, sort_keys=True))


def receipt(observed: dict[str, tuple[str, Optional[str]]]) -> dict:
    rows = [
        {"id": case_id, "disposition": disposition, "reason": reason}
        for case_id, (disposition, reason) in sorted(observed.items())
    ]
    return {
        "schema": "sup-civ-000d1a-reference-receipt-v1",
        "model_schema": SCHEMA,
        "digest_authority": DIGEST_AUTHORITY,
        "case_count": len(rows),
        "admit_count": sum(1 for row in rows if row["disposition"] == "Admit"),
        "refuse_count": sum(1 for row in rows if row["disposition"] == "Refuse"),
        "cases": rows,
        "nonclaims": [
            "no cryptographic primitive executed",
            "reference-model digest is not a runtime envelope id",
            "recipient listing is not current authorization",
            "successful semantic transition is not access legitimacy",
            "key rotation is not retroactive knowledge revocation",
            "storage topology is not selected",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--receipt", type=Path)
    args = parser.parse_args()

    observed = build_cases()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    validate_manifest(manifest, observed)
    result = receipt(observed)
    encoded = json.dumps(result, sort_keys=True, indent=2) + "\n"
    if args.receipt:
        args.receipt.write_text(encoded, encoding="utf-8")
    else:
        sys.stdout.write(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
