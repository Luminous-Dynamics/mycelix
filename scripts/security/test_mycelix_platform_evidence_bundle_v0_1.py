#!/usr/bin/env python3
"""End-to-end reference-model test for the platform Evidence bundle boundary."""
from __future__ import annotations

import hashlib
import json
import struct
import os
import subprocess
import sys
import tempfile
from pathlib import Path

SECURITY = Path(__file__).resolve().parent
ROOT = SECURITY.parents[1]
sys.path.insert(0, str(SECURITY))

import verify_mycelix_platform_evidence_capture_v0_1 as platform  # noqa: E402



def make_valid_eventlog() -> tuple[bytes, list[tuple[int, int, bytes]]]:
    signature = b"Spec ID Event03" + b"\x00"
    spec = (
        signature
        + struct.pack("<I", 0)
        + bytes([0, 2, 0, 2])
        + struct.pack("<I", 2)
        + struct.pack("<HH", 0x0004, 20)
        + struct.pack("<HH", 0x000B, 32)
        + bytes([0])
    )
    legacy = (
        struct.pack("<II", 0, 0x00000003)
        + b"\x00" * 20
        + struct.pack("<I", len(spec))
        + spec
    )
    events = [
        (4, 0x80000003, b"firmware-app"),
        (4, 0x00000004, b"\x00\x00\x00\x00"),
        (7, 0x800000E0, b"variable-authority"),
        (0, 0x00000005, b"action-zero"),
        (2, 0x00000005, b"action-two"),
    ]
    encoded = bytearray(legacy)
    for pcr, event_type, payload in events:
        encoded.extend(struct.pack("<III", pcr, event_type, 1))
        encoded.extend(struct.pack("<H", 0x000B))
        encoded.extend(hashlib.sha256(payload).digest())
        encoded.extend(struct.pack("<I", len(payload)))
        encoded.extend(payload)
    return bytes(encoded), events


def run_reconstruction(input_path: Path, output_path: Path) -> None:
    proc = subprocess.run(
        [
            sys.executable,
            str(SECURITY / "reconstruct_mycelix_pc_client_eventlog_v0_1.py"),
            "--reconstruct",
            str(input_path),
            "--output",
            str(output_path),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(proc.stdout + proc.stderr)


def build_bundle(bundle: Path) -> dict:
    (bundle / "reference-values.json").write_text(
        '{"version":"pc-client-rim-2026.1","rules":"reference-model-fixture"}\n',
        encoding="utf-8",
    )
    (bundle / "trusted-time.json").write_text(
        '{"available":true,"policy_trusted":true,"local_clock_only":false}\n',
        encoding="utf-8",
    )
    (bundle / "nonce.bin").write_bytes(b"external-verifier-nonce-v1")
    (bundle / "tss-version-evidence.txt").write_text(
        "TSS2-TCTI reference-model fixture\n",
        encoding="utf-8",
    )
    (bundle / "tpm-properties.txt").write_text(
        "TPM2_PT_MANUFACTURER: FIXTURE\nTPM2_PT_VENDOR_STRING_1: MODEL\n",
        encoding="utf-8",
    )
    eventlog_bytes, encoded_events = make_valid_eventlog()
    (bundle / "eventlog.bin").write_bytes(eventlog_bytes)
    (bundle / "eventlog-parsed.yaml").write_text(
        "fixture: parser-pass\n",
        encoding="utf-8",
    )
    (bundle / "quote.msg").write_bytes(b"quote-message-fixture")
    (bundle / "quote.sig").write_bytes(b"quote-signature-fixture")
    public_body = bytes.fromhex("0001000b") + bytes(range(1, 65))
    public_wrapper = len(public_body).to_bytes(2, "big") + public_body
    (bundle / "ak.pub").write_bytes(public_wrapper)
    (bundle / "ek.pub").write_bytes(public_wrapper)

    for role in ("ak", "ek"):
        (bundle / f"{role}.tpmt").write_bytes(public_body)
        (bundle / f"{role}.name.readpublic").write_bytes(
            b"\x00\x0b" + hashlib.sha256(public_body).digest()
        )
        (bundle / f"{role}.qname.readpublic").write_bytes(
            b"\x00\x0b" + hashlib.sha256(b"qname:" + public_body).digest()
        )
        transcript = {
            "role": role,
            "command": [
                "tpm2_readpublic", "-Q", "-c", f"{role}.ctx", "-f", "tpmt",
                "-o", f"{role}.tpmt", "-n", f"{role}.name.readpublic",
                "-q", f"{role}.qname.readpublic",
            ],
            "tool_version": "5.8 fixture",
            "returncode": 0,
            "stdout": "",
            "stderr": "",
        }
        (bundle / f"{role}.readpublic-transcript.json").write_text(
            json.dumps(transcript, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        public_input = {
            "profile_id": "mycelix.security.tpm.public-name-coherence",
            "profile_version": "0.1.0",
            "verification_mode": "LiveVerifierSession",
            "claim_ceiling": "ReferenceModelOnly",
            "object_role": role.upper(),
            "public_format": "TPMT_PUBLIC",
            "public_wire_hex": public_body.hex(),
            "public_wire_sha256": hashlib.sha256(public_body).hexdigest(),
            "name_hex": (b"\x00\x0b" + hashlib.sha256(public_body).digest()).hex(),
            "readpublic_state": "PASS",
            "readpublic_source_sha256": hashlib.sha256(
                (bundle / f"{role}.readpublic-transcript.json").read_bytes()
            ).hexdigest(),
        }
        (bundle / f"{role}-public-name-input.json").write_text(
            json.dumps(public_input, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        public_result = subprocess.run(
            [
                sys.executable,
                str(SECURITY / "verify_mycelix_tpm_public_name_coherence_v0_1.py"),
                "--verify", str(bundle / f"{role}-public-name-input.json"),
                "--output", str(bundle / f"{role}-public-name-coherence.json"),
            ],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
        )
        if public_result.returncode != 2:
            raise RuntimeError(
                f"public-name fixture for {role} returned {public_result.returncode}: "
                f"{public_result.stdout}{public_result.stderr}"
            )
    (bundle / "tool-versions.json").write_text(
        json.dumps(
            {name: "5.8 fixture" for name in (
                "tpm2_getcap",
                "tpm2_pcrread",
                "tpm2_quote",
                "tpm2_createek",
                "tpm2_createak",
                "tpm2_checkquote",
                "tpm2_eventlog",
                "tpm2_readpublic",
            )},
            indent=2,
            sort_keys=True,
        ) + "\n",
        encoding="utf-8",
    )

    event_stream = {
        "profile_id": "mycelix.security.platform.eventlog.reconstruction",
        "profile_version": "0.1.0",
        "event_log_sha256": platform.sha256_file(bundle / "eventlog.bin"),
        "session_id": "self-test-session",
        "pcr_bank": "sha256",
        "pcr_selection": "sha256:0,2,4,7",
        "events": [
            {"sequence": 1, "pcr": encoded_events[0][0], "event_type": "EV_EFI_BOOT_SERVICES_APPLICATION", "digest_sha256": hashlib.sha256(encoded_events[0][2]).hexdigest(), "session_id": "self-test-session"},
            {"sequence": 2, "pcr": encoded_events[1][0], "event_type": "EV_SEPARATOR", "digest_sha256": hashlib.sha256(encoded_events[1][2]).hexdigest(), "session_id": "self-test-session"},
            {"sequence": 3, "pcr": encoded_events[2][0], "event_type": "EV_EFI_VARIABLE_AUTHORITY", "digest_sha256": hashlib.sha256(encoded_events[2][2]).hexdigest(), "session_id": "self-test-session"},
            {"sequence": 4, "pcr": encoded_events[3][0], "event_type": "EV_ACTION", "digest_sha256": hashlib.sha256(encoded_events[3][2]).hexdigest(), "session_id": "self-test-session"},
            {"sequence": 5, "pcr": encoded_events[4][0], "event_type": "EV_ACTION", "digest_sha256": hashlib.sha256(encoded_events[4][2]).hexdigest(), "session_id": "self-test-session"},
        ],
    }
    input_path = bundle / "eventlog-reconstruction-input.json"
    input_path.write_text(json.dumps(event_stream, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    reconstruction_path = bundle / "eventlog-reconstruction.json"
    run_reconstruction(input_path, reconstruction_path)
    reconstruction = json.loads(reconstruction_path.read_text(encoding="utf-8"))

    observed_pcr_path = bundle / "observed-pcr-values.json"
    observed_pcr_path.write_text(
        json.dumps(reconstruction["observed_pcr_values"], indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    raw_eventlog_path = bundle / "raw-eventlog.json"
    raw_proc = subprocess.run(
        [
            sys.executable,
            str(SECURITY / "parse_mycelix_raw_tpm2_eventlog_v0_1.py"),
            "--parse",
            str(bundle / "eventlog.bin"),
            "--output",
            str(raw_eventlog_path),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if raw_proc.returncode != 0:
        raise RuntimeError(raw_proc.stdout + raw_proc.stderr)

    payload_coherence_path = bundle / "payload-coherence.json"
    payload_proc = subprocess.run(
        [
            sys.executable,
            str(SECURITY / "verify_mycelix_event_payload_digest_coherence_v0_1.py"),
            "--verify",
            str(input_path),
            "--output",
            str(payload_coherence_path),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if payload_proc.returncode not in (0, 2):
        raise RuntimeError(payload_proc.stdout + payload_proc.stderr)
    payload_coherence = json.loads(payload_coherence_path.read_text(encoding="utf-8"))

    values = reconstruction["observed_pcr_values"]
    (bundle / "pcr-post.yaml").write_text(
        "sha256:\n" + "".join(
            f"  {index}: {values[index]}\n" for index in sorted(values, key=int)
        ),
        encoding="utf-8",
    )

    manifest = platform.fixture_manifest()
    for role in ("ak", "ek"):
        manifest["public_name_coherence"][role] = {
            "status": "INDETERMINATE",
            "verifier_id": "mycelix.tpm.public-name-coherence.v0.1",
            "output_sha256": platform.sha256_file(bundle / f"{role}-public-name-coherence.json"),
            "input_sha256": platform.sha256_file(bundle / f"{role}-public-name-input.json"),
            "wire_sha256": platform.sha256_file(bundle / f"{role}.tpmt"),
            "name_sha256": platform.sha256_file(bundle / f"{role}.name.readpublic"),
            "qname_sha256": platform.sha256_file(bundle / f"{role}.qname.readpublic"),
            "source_sha256": platform.sha256_file(bundle / f"{role}.readpublic-transcript.json"),
        }
    manifest["session_id"] = event_stream["session_id"]
    manifest["boot_id"] = "self-test-boot"
    manifest["tpm"]["properties_sha256"] = platform.sha256_file(bundle / "tpm-properties.txt")
    manifest["tpm"]["ek_public_sha256"] = platform.sha256_file(bundle / "ek.pub")
    manifest["event_log"]["sha256"] = platform.sha256_file(bundle / "eventlog.bin")
    manifest["event_log"]["parser_output_sha256"] = platform.sha256_file(bundle / "eventlog-parsed.yaml")
    manifest["raw_eventlog"] = {
        "status": "PASS",
        "parser_id": "mycelix.pc-client.raw-tpm2-eventlog-parser.v0.1",
        "output_sha256": platform.sha256_file(raw_eventlog_path),
        "source_sha256": platform.sha256_file(
            SECURITY / "parse_mycelix_raw_tpm2_eventlog_v0_1.py"
        ),
        "binary_sha256": platform.sha256_file(bundle / "eventlog.bin"),
    }
    manifest["quote"]["nonce_sha256"] = platform.sha256_file(bundle / "nonce.bin")
    manifest["quote"]["attestation_key_sha256"] = platform.sha256_file(bundle / "ak.pub")
    manifest["challenge"]["sha256"] = platform.sha256_file(bundle / "nonce.bin")
    manifest["toolchain"]["observed_tool_versions_sha256"] = platform.sha256_file(bundle / "tool-versions.json")
    manifest["reference_values"]["sha256"] = platform.sha256_file(bundle / "reference-values.json")
    manifest["trusted_time"]["sha256"] = platform.sha256_file(bundle / "trusted-time.json")
    manifest["reconstruction"] = {
        "status": reconstruction["reconstruction_status"],
        "event_log_sha256": reconstruction["event_log_sha256"],
        "pcr_selection": reconstruction["pcr_selection"],
        "reconstructed_pcrs_sha256": reconstruction["reconstructed_pcrs_sha256"],
        "content_sha256": reconstruction["content_sha256"],
        "input_sha256": platform.sha256_file(input_path),
        "verifier_id": reconstruction["verifier_id"],
        "verifier_source_sha256": reconstruction["verifier_source_sha256"],
    }
    manifest["payload_coherence"] = {
        "status": payload_coherence["state"],
        "output_sha256": platform.sha256_file(payload_coherence_path),
        "source_sha256": platform.sha256_file(
            SECURITY / "verify_mycelix_event_payload_digest_coherence_v0_1.py"
        ),
        "input_sha256": platform.sha256_file(input_path),
    }
    manifest["live_observation"]["selection"] = reconstruction["pcr_selection"]
    manifest["live_observation"]["pcr_values_sha256"] = reconstruction["observed_pcrs_sha256"]
    manifest["live_observation"]["pcr_post_artifact_sha256"] = platform.sha256_file(bundle / "pcr-post.yaml")
    manifest["live_observation"]["pcr_values_file_sha256"] = platform.sha256_file(observed_pcr_path)
    manifest["payload_coherence"] = {
        "status": payload_coherence["state"],
        "output_sha256": platform.sha256_file(payload_coherence_path),
        "source_sha256": platform.sha256_file(
            SECURITY / "verify_mycelix_event_payload_digest_coherence_v0_1.py"
        ),
        "input_sha256": platform.sha256_file(input_path),
    }
    manifest["artifacts"]["quote_message_sha256"] = platform.sha256_file(bundle / "quote.msg")
    manifest["artifacts"]["quote_signature_sha256"] = platform.sha256_file(bundle / "quote.sig")
    manifest["artifacts"]["attestation_key_sha256"] = platform.sha256_file(bundle / "ak.pub")
    manifest["artifacts"]["reconstruction_file_sha256"] = platform.sha256_file(reconstruction_path)
    manifest["artifacts"]["public_name_ak_output_sha256"] = platform.sha256_file(bundle / "ak-public-name-coherence.json")
    manifest["artifacts"]["public_name_ak_input_sha256"] = platform.sha256_file(bundle / "ak-public-name-input.json")
    manifest["artifacts"]["public_name_ak_wire_sha256"] = platform.sha256_file(bundle / "ak.tpmt")
    manifest["artifacts"]["public_name_ak_name_sha256"] = platform.sha256_file(bundle / "ak.name.readpublic")
    manifest["artifacts"]["public_name_ak_qname_sha256"] = platform.sha256_file(bundle / "ak.qname.readpublic")
    manifest["artifacts"]["public_name_ak_source_sha256"] = platform.sha256_file(bundle / "ak.readpublic-transcript.json")
    manifest["artifacts"]["public_name_ek_source_sha256"] = platform.sha256_file(bundle / "ek.readpublic-transcript.json")
    manifest["artifacts"]["public_name_ek_output_sha256"] = platform.sha256_file(bundle / "ek-public-name-coherence.json")
    manifest["artifacts"]["public_name_ek_input_sha256"] = platform.sha256_file(bundle / "ek-public-name-input.json")
    manifest["artifacts"]["public_name_ek_wire_sha256"] = platform.sha256_file(bundle / "ek.tpmt")
    manifest["artifacts"]["public_name_ek_name_sha256"] = platform.sha256_file(bundle / "ek.name.readpublic")
    manifest["artifacts"]["public_name_ek_qname_sha256"] = platform.sha256_file(bundle / "ek.qname.readpublic")
    manifest["artifacts"]["reconstruction_input_sha256"] = platform.sha256_file(input_path)
    manifest["artifacts"]["observed_pcr_values_file_sha256"] = platform.sha256_file(observed_pcr_path)
    manifest["artifacts"]["raw_eventlog_output_sha256"] = platform.sha256_file(raw_eventlog_path)
    manifest["artifacts"]["payload_coherence_output_sha256"] = platform.sha256_file(payload_coherence_path)
    manifest["artifacts"]["raw_eventlog_output_sha256"] = platform.sha256_file(raw_eventlog_path)
    manifest["artifacts"]["payload_coherence_output_sha256"] = platform.sha256_file(payload_coherence_path)
    manifest["artifacts"]["tss_version_evidence_sha256"] = platform.sha256_file(bundle / "tss-version-evidence.txt")
    manifest["artifacts"]["ek_public_sha256"] = platform.sha256_file(bundle / "ek.pub")
    manifest["os_image_digest"] = "sha256:" + "a1" * 32
    manifest["workload_digest"] = "sha256:" + "b2" * 32
    manifest["tpm"]["identity_digest"] = platform.canonical_hash({
        "device_path": manifest["tpm"]["device_path"],
        "properties_sha256": manifest["tpm"]["properties_sha256"],
        "ek_public_sha256": manifest["tpm"]["ek_public_sha256"],
    })
    manifest["session_binding_sha256"] = platform.session_binding(manifest)
    (bundle / "capture-session.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def stub_quote_checker(bundle: Path):
    stub_dir = bundle / "stub-bin"
    stub_dir.mkdir()
    marker = bundle / "quote-check-ran"
    (stub_dir / "tpm2_checkquote").write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "from pathlib import Path\n"
        "Path(os.environ['MYCELIX_QUOTE_STUB_MARKER']).touch()\n",
        encoding="utf-8",
    )
    (stub_dir / "tpm2_checkquote").chmod(0o755)
    old_path = os.environ.get("PATH", "")
    old_marker = os.environ.get("MYCELIX_QUOTE_STUB_MARKER")
    os.environ["PATH"] = str(stub_dir) + os.pathsep + old_path
    os.environ["MYCELIX_QUOTE_STUB_MARKER"] = str(marker)
    return old_path, old_marker, marker


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="mycelix-platform-bundle-test-") as td:
        bundle = Path(td)
        build_bundle(bundle)
        old_path, old_marker, marker = stub_quote_checker(bundle)
        try:
            first = platform.verify_bundle(type("Args", (), {"bundle": str(bundle)})())
            if first != 0 or not marker.is_file():
                print("End-to-end reference-model bundle: FAIL")
                return 1

            raw_path = bundle / "raw-eventlog.json"
            reconstruction_path = bundle / "eventlog-reconstruction.json"
            pcr_path = bundle / "pcr-post.yaml"
            manifest_path = bundle / "capture-session.json"
            reconstruction = json.loads(reconstruction_path.read_text(encoding="utf-8"))
            reconstruction["reconstructed_pcr_values"]["4"] = "ff" * 32
            reconstruction["observed_pcr_values"]["4"] = "ff" * 32
            reconstruction["reconstructed_pcrs_sha256"] = platform.pcr_values_hash(
                reconstruction["reconstructed_pcr_values"]
            )
            reconstruction["observed_pcrs_sha256"] = platform.pcr_values_hash(
                reconstruction["observed_pcr_values"]
            )
            reconstruction["content_sha256"] = platform.self_hash(
                reconstruction, "content_sha256"
            )
            reconstruction_path.write_text(
                json.dumps(reconstruction, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            (bundle / "pcr-post.yaml").write_text(
                "sha256:\n" + "".join(
                    f"  {index}: {reconstruction['observed_pcr_values'][index]}\n"
                    for index in sorted(reconstruction["observed_pcr_values"], key=int)
                ),
                encoding="utf-8",
            )

            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["live_observation"]["pcr_values_sha256"] = reconstruction["observed_pcrs_sha256"]
            manifest["live_observation"]["pcr_post_artifact_sha256"] = platform.sha256_file(pcr_path)
            manifest["reconstruction"]["reconstructed_pcrs_sha256"] = reconstruction["reconstructed_pcrs_sha256"]
            manifest["reconstruction"]["content_sha256"] = reconstruction["content_sha256"]
            manifest["artifacts"]["reconstruction_file_sha256"] = platform.sha256_file(reconstruction_path)
            manifest["session_binding_sha256"] = platform.session_binding(manifest)
            manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

            second = platform.verify_bundle(type("Args", (), {"bundle": str(bundle)})())
            if second == 0:
                print("False-green independent-reconstruction regression: FAIL")
                return 1

            # The raw sidecar must not be able to self-certify. Re-hash an altered
            # sidecar so manifest/file-digest checks still pass; re-execution must
            # nevertheless detect that it differs from the binary parser result.
            raw = json.loads(raw_path.read_text(encoding="utf-8"))
            raw["events"][1]["payload_hex"] = "deadbeef"
            raw["events"][1]["digest_sha256"] = hashlib.sha256(bytes.fromhex("deadbeef")).hexdigest()
            raw["events"][1]["digests"]["sha256"] = raw["events"][1]["digest_sha256"]
            raw["content_sha256"] = platform.self_hash(raw, "content_sha256")
            raw_path.write_text(json.dumps(raw, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["raw_eventlog"]["output_sha256"] = platform.sha256_file(raw_path)
            manifest["artifacts"]["raw_eventlog_output_sha256"] = platform.sha256_file(raw_path)
            manifest["session_binding_sha256"] = platform.session_binding(manifest)
            manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            third = platform.verify_bundle(type("Args", (), {"bundle": str(bundle)})())
            if third == 0:
                print("False-green raw-parser replay regression: FAIL")
                return 1

            print("End-to-end reference-model bundle: PASS")
            print("False-green independent-reconstruction regression: PASS")
            return 0
        finally:
            os.environ["PATH"] = old_path
            if old_marker is None:
                os.environ.pop("MYCELIX_QUOTE_STUB_MARKER", None)
            else:
                os.environ["MYCELIX_QUOTE_STUB_MARKER"] = old_marker


if __name__ == "__main__":
    raise SystemExit(main())
