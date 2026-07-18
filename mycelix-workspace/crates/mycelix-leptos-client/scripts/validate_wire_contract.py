#!/usr/bin/env python3
"""Dependency-free guard for the browser conductor and signing contract.

This is intentionally static. It catches high-risk source drift before the
Rust/WASM and real-conductor lanes run; it does not claim runtime compatibility.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TYPES = ROOT / "src/types.rs"
BROWSER = ROOT / "src/browser.rs"
NATIVE = ROOT / "src/native.rs"
SIGNING_DOC = ROOT / "docs/browser-signing.md"
LIFECYCLE_DOC = ROOT / "docs/transport-lifecycle.md"
WIRE_TEST = ROOT / "tests/wire_contract.rs"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def main() -> int:
    errors: list[str] = []
    sources = {
        TYPES: read(TYPES),
        BROWSER: read(BROWSER),
        NATIVE: read(NATIVE),
        SIGNING_DOC: read(SIGNING_DOC),
        LIFECYCLE_DOC: read(LIFECYCLE_DOC),
        WIRE_TEST: read(WIRE_TEST),
    }

    required = {
        TYPES: {
            'content = "value"',
            "serializer.serialize_bytes",
            "AppInfo(Option<AppInfoResponse>)",
            "CallZome(SignedZomeCall)",
            "IncomingWireMessage::Signal",
            "82a474797065a86170705f696e666fa576616c7565c0",
            "81a5746f6b656ec403010203",
        },
        BROWSER: {
            "__HC_ZOME_CALL_SIGNER__",
            "signZomeCall",
            "zome_call_signer_available",
            "SignedZomeCall::new",
            "Holochain 0.6 app WebSockets require an app authentication token",
            "crypto.getRandomValues",
            "failed to decode call_zome response",
            "set_status_handler",
            "publish_status",
            "handle_disconnect",
            "reconnect_scheduled",
            "reconnect_epoch",
        },
        NATIVE: {"SigningUnavailable", "WireAuthenticate"},
        SIGNING_DOC: {
            "provenance: Uint8Array",
            "signature: Uint8Array(64)",
            "Raw private keys do not cross",
        },
        LIFECYCLE_DOC: {
            "set_status_handler",
            "Duplicate",
            "disconnect()",
            "zome_call_signer_available()",
            "startup retry window",
            "retry epoch",
        },
        WIRE_TEST: {
            "signed_call_rejects_empty_bytes",
            "signed_call_rejects_wrong_signature_length",
            "signed_call_rejects_zero_signature",
        },
    }
    for path, fragments in required.items():
        for fragment in sorted(fragments):
            if fragment not in sources[path]:
                errors.append(f"{path.relative_to(ROOT)} is missing {fragment!r}")

    production = "\n".join((sources[TYPES], sources[BROWSER], sources[NATIVE]))
    forbidden = {
        r'content\s*=\s*"data"': "legacy type/data app envelope",
        r"Math::random": "non-cryptographic nonce fallback",
        r"CallZomeRequestWire": "legacy unsigned call DTO",
        r"WireResponse": "legacy signal-as-response DTO",
        r"signature\s*:\s*vec!\[0(?:u8)?\s*;\s*64\]": "constructed zero signature",
        r"Err\(_\)\s*=>\s*Ok\(response_bytes\)": "raw malformed-response success fallback",
    }
    for pattern, description in forbidden.items():
        if re.search(pattern, production):
            errors.append(f"production source contains {description}")

    status_assignments = re.findall(r"state\.status\s*=(?!=)", sources[BROWSER])
    if len(status_assignments) != 1:
        errors.append(
            "browser status must be assigned only by publish_status; "
            f"found {len(status_assignments)} assignment sites"
        )

    misleading_test = ROOT / "tests/conductor_e2e.rs"
    if misleading_test.exists():
        errors.append(
            "tests/conductor_e2e.rs exists; an official-client-only test must not be "
            "presented as validation of this transport"
        )

    if errors:
        print("Browser wire contract validation failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print(
        "Browser wire contract valid: type/value DTOs, binary fields, required auth, "
        "explicit signer, secure nonce, and fail-closed responses present."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
