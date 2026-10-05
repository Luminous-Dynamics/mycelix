#!/usr/bin/env python3
"""Capture and deterministically parse TPM2_GetCapability properties-fixed output."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.properties-fixed-capture.v0.1"

REQUIRED_PROPERTIES = (
    "TPM2_PT_MANUFACTURER",
    "TPM2_PT_VENDOR_TPM_TYPE",
    "TPM2_PT_VENDOR_STRING_1",
    "TPM2_PT_VENDOR_STRING_2",
    "TPM2_PT_VENDOR_STRING_3",
    "TPM2_PT_VENDOR_STRING_4",
    "TPM2_PT_FIRMWARE_VERSION_1",
    "TPM2_PT_FIRMWARE_VERSION_2",
)

PROPERTY_ALIASES = {
    name: (name, name.replace("TPM2_PT_", "TPM_PT_"))
    for name in REQUIRED_PROPERTIES
}


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
        ).encode("utf-8")
    ).hexdigest()


def valid_hash(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(char in "0123456789abcdef" for char in value)
    )


def parse_properties(text: str) -> dict[str, dict[str, str]]:
    lines = text.splitlines()
    parsed: dict[str, dict[str, str]] = {}

    property_header_re = re.compile(r"^(TPM2?_PT_[A-Z0-9_]+):$")

    for property_name in REQUIRED_PROPERTIES:
        aliases = set(PROPERTY_ALIASES[property_name])
        starts = [
            index
            for index, line in enumerate(lines)
            if line.strip().rstrip(":") in aliases
            and line.strip().endswith(":")
        ]
        if len(starts) != 1:
            raise ValueError(
                f"expected exactly one block for {property_name}, observed {len(starts)}"
            )

        start = starts[0]
        block_lines: list[str] = []
        for line in lines[start + 1 :]:
            stripped = line.strip()
            if property_header_re.match(stripped):
                break
            block_lines.append(line)

        block = "\n".join(block_lines)
        raw_match = re.search(
            r"\braw:\s*0x([0-9a-fA-F]{1,8})\b",
            block,
        )
        if not raw_match:
            raise ValueError(f"missing raw field for {property_name}")

        value_match = re.search(
            r'\bvalue:\s*"([^"]*)"',
            block,
        )

        parsed[property_name] = {
            "raw_hex": raw_match.group(1).upper().rjust(8, "0"),
            "value": value_match.group(1) if value_match else "",
        }

    return parsed


def result(
    state: str,
    reason: str,
    details: dict[str, Any] | None = None,
) -> dict[str, Any]:
    output: dict[str, Any] = {
        "verifier_id": VERIFIER_ID,
        "state": state,
        "reason": reason,
    }
    if details is not None:
        output["details"] = details
    return output


def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    required = {
        "profile_id",
        "profile_version",
        "claim_ceiling",
        "verification_mode",
        "command",
        "source_output",
        "source_output_sha256",
        "parsed_properties",
    }
    missing = sorted(required - set(manifest))
    if missing:
        return result("DENY", "missing-required-fields", {"fields": missing})

    if manifest["profile_id"] != "mycelix.security.tpm.properties-fixed-capture":
        return result("DENY", "profile-id-mismatch")

    if manifest["profile_version"] != "0.1.0":
        return result("DENY", "profile-version-mismatch")

    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        return result("DENY", "claim-ceiling-mismatch")

    if manifest["verification_mode"] not in {
        "ReferenceModelOnly",
        "OfflineBundle",
        "LiveVerifierSession",
    }:
        return result("DENY", "verification-mode-invalid")

    if manifest["command"] != ["tpm2_getcap", "properties-fixed"]:
        return result("DENY", "command-not-properties-fixed")

    if not isinstance(manifest["source_output"], str):
        return result("DENY", "source-output-not-text")

    if not valid_hash(manifest["source_output_sha256"]):
        return result("DENY", "source-output-digest-invalid")

    actual_source_hash = hashlib.sha256(
        manifest["source_output"].encode("utf-8")
    ).hexdigest()
    if actual_source_hash != manifest["source_output_sha256"]:
        return result("DENY", "source-output-digest-mismatch")

    try:
        parsed = parse_properties(manifest["source_output"])
    except ValueError as exc:
        return result(
            "DENY",
            "properties-parse-failed",
            {"error": str(exc)},
        )

    if parsed != manifest["parsed_properties"]:
        return result("DENY", "parsed-property-substitution")

    if manifest["verification_mode"] != "ReferenceModelOnly":
        return result(
            "INDETERMINATE",
            "live-property-source-not-authorized-by-reference-model",
            {"parsed_properties": parsed},
        )

    return result(
        "PASS",
        "tpm-fixed-properties-captured-and-reparsed",
        {"parsed_properties": parsed},
    )


def fixture() -> dict[str, Any]:
    source = "\n".join(
        [
            "TPM2_PT_MANUFACTURER:",
            "  raw: 0x4D594358",
            '  value: "MYCX"',
            "TPM2_PT_VENDOR_TPM_TYPE:",
            "  raw: 0x00000000",
            "  value: \"\"",
            "TPM2_PT_VENDOR_STRING_1:",
            "  raw: 0x53594E54",
            '  value: "SYNT"',
            "TPM2_PT_VENDOR_STRING_2:",
            "  raw: 0x48455449",
            '  value: "HETI"',
            "TPM2_PT_VENDOR_STRING_3:",
            "  raw: 0x00000000",
            '  value: ""',
            "TPM2_PT_VENDOR_STRING_4:",
            "  raw: 0x00000000",
            '  value: ""',
            "TPM2_PT_FIRMWARE_VERSION_1:",
            "  raw: 0x00000000",
            "  value: \"\"",
            "TPM2_PT_FIRMWARE_VERSION_2:",
            "  raw: 0x00010002",
            "  value: \"\"",
        ]
    )
    parsed = parse_properties(source)
    return {
        "profile_id": "mycelix.security.tpm.properties-fixed-capture",
        "profile_version": "0.1.0",
        "claim_ceiling": "ReferenceModelOnly",
        "verification_mode": "ReferenceModelOnly",
        "command": ["tpm2_getcap", "properties-fixed"],
        "source_output": source,
        "source_output_sha256": hashlib.sha256(source.encode("utf-8")).hexdigest(),
        "parsed_properties": parsed,
    }


def self_test() -> int:
    base = fixture()

    def recompute_source_hash(candidate: dict[str, Any]) -> None:
        candidate["source_output_sha256"] = hashlib.sha256(
            candidate["source_output"].encode("utf-8")
        ).hexdigest()

    cases = [
        ("canonical-capture", "PASS", lambda candidate: None),
        (
            "manufacturer-raw-substitution",
            "DENY",
            lambda candidate: candidate.update(
                {
                    "source_output": candidate["source_output"].replace(
                        "0x4D594358", "0x4D594359", 1
                    )
                }
            ),
        ),
        (
            "vendor-type-raw-substitution",
            "DENY",
            lambda candidate: candidate.update(
                {
                    "source_output": candidate["source_output"].replace(
                        "TPM2_PT_VENDOR_TPM_TYPE:\n  raw: 0x00000000",
                        "TPM2_PT_VENDOR_TPM_TYPE:\n  raw: 0x00000001",
                        1,
                    )
                }
            ),
        ),
        (
            "firmware-v1-raw-substitution",
            "DENY",
            lambda candidate: candidate.update(
                {
                    "source_output": candidate["source_output"].replace(
                        "TPM2_PT_FIRMWARE_VERSION_1:\n  raw: 0x00000000",
                        "TPM2_PT_FIRMWARE_VERSION_1:\n  raw: 0x00000001",
                        1,
                    )
                }
            ),
        ),
        (
            "firmware-v2-raw-substitution",
            "DENY",
            lambda candidate: candidate.update(
                {
                    "source_output": candidate["source_output"].replace(
                        "0x00010002", "0x00010003", 1
                    )
                }
            ),
        ),
        (
            "required-property-missing",
            "DENY",
            lambda candidate: candidate.update(
                {
                    "source_output": candidate["source_output"].replace(
                        "TPM2_PT_VENDOR_STRING_4:\n  raw: 0x00000000\n  value: \"\"\n",
                        "",
                        1,
                    )
                }
            ),
        ),
        (
            "duplicate-property",
            "DENY",
            lambda candidate: candidate.update(
                {
                    "source_output": candidate["source_output"]
                    + "\nTPM2_PT_MANUFACTURER:\n  raw: 0x4D594358\n"
                }
            ),
        ),
        (
            "source-digest-substitution",
            "DENY",
            lambda candidate: candidate.update(
                {"source_output_sha256": "77" * 32}
            ),
        ),
        (
            "parsed-result-substitution",
            "DENY",
            lambda candidate: candidate.update(
                {"parsed_properties": {}}
            ),
        ),
        (
            "command-substitution",
            "DENY",
            lambda candidate: candidate.update(
                {"command": ["host_tool", "properties-fixed"]}
            ),
        ),
        (
            "unknown-property-allowed",
            "PASS",
            lambda candidate: (
                candidate.update(
                    {
                        "source_output": candidate["source_output"]
                        + '\nTPM2_PT_UNKNOWN:\n  raw: 0x12345678\n',
                    }
                ),
                recompute_source_hash(candidate),
            ),
        ),
        (
            "live-unintegrated",
            "INDETERMINATE",
            lambda candidate: candidate.update(
                {"verification_mode": "LiveVerifierSession"}
            ),
        ),
    ]

    for name, expected, mutation in cases:
        candidate = copy.deepcopy(base)
        mutation(candidate)
        observed = verify(candidate)
        if observed["state"] != expected:
            print(
                f"{name}: FAIL expected={expected} "
                f"got={observed['state']} reason={observed['reason']}"
            )
            return 1

    if verify(json.loads(json.dumps(base, sort_keys=True)))["state"] != "PASS":
        print("key-order-permutation: FAIL")
        return 1

    print("TPM properties-fixed capture semantic corpus: PASS")
    print("11 adversarial/canonical vectors plus key-order control: PASS")
    print("raw command, source digest, parser output, and live-origin boundaries are enforced")
    return 0


def capture(out: Path) -> int:
    out.mkdir(parents=True, exist_ok=True)
    raw_path = out / "tpm-properties-fixed.txt"
    result_path = out / "tpm-properties-fixed.json"
    transcript_path = out / "tpm-properties-fixed-transcript.json"

    if shutil.which("tpm2_getcap") is None:
        value = result(
            "INDETERMINATE",
            "tpm2_getcap-unavailable",
        )
        result_path.write_text(
            json.dumps(value, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return 2

    command = ["tpm2_getcap", "properties-fixed"]
    process = subprocess.run(
        command,
        cwd=out,
        env=os.environ.copy(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    raw_path.write_text(process.stdout, encoding="utf-8")
    transcript = {
        "command": command,
        "returncode": process.returncode,
        "stdout_sha256": hashlib.sha256(
            process.stdout.encode("utf-8")
        ).hexdigest(),
        "stderr_sha256": hashlib.sha256(
            process.stderr.encode("utf-8")
        ).hexdigest(),
    }
    transcript_path.write_text(
        json.dumps(transcript, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    if process.returncode != 0:
        value = result(
            "INDETERMINATE",
            "tpm2_getcap-properties-fixed-failed",
            {
                "returncode": process.returncode,
                "stderr_sha256": transcript["stderr_sha256"],
                "raw_sha256": hashlib.sha256(
                    process.stdout.encode("utf-8")
                ).hexdigest(),
            },
        )
        result_path.write_text(
            json.dumps(value, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return 2

    try:
        parsed = parse_properties(process.stdout)
    except ValueError as exc:
        value = result(
            "DENY",
            "tpm-properties-fixed-live-parse-failed",
            {"error": str(exc)},
        )
        result_path.write_text(
            json.dumps(value, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return 1

    value = result(
        "PASS",
        "tpm-fixed-properties-captured",
        {
            "command": command,
            "source_output_sha256": hashlib.sha256(
                process.stdout.encode("utf-8")
            ).hexdigest(),
            "parsed_properties": parsed,
            "transcript_sha256": hashlib.sha256(
                transcript_path.read_bytes()
            ).hexdigest(),
        },
    )
    result_path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true")
    group.add_argument("--capture", action="store_true")
    parser.add_argument("--output")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    if not args.output:
        parser.error("--output is required with --capture")

    return capture(Path(args.output).resolve())


if __name__ == "__main__":
    raise SystemExit(main())
