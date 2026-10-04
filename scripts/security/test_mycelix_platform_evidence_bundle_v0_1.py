#!/usr/bin/env python3
"""End-to-end reference-model test for the platform Evidence bundle boundary."""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

SECURITY = Path(__file__).resolve().parent
ROOT = SECURITY.parents[1]
sys.path.insert(0, str(SECURITY))

import verify_mycelix_platform_evidence_capture_v0_1 as platform  # noqa: E402


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
    (bundle / "eventlog.bin").write_bytes(b"PC-CLIENT-EVENTLOG-FIXTURE-V1\n")
    (bundle / "eventlog-parsed.yaml").write_text(
        "fixture: parser-pass\n",
        encoding="utf-8",
    )
    (bundle / "quote.msg").write_bytes(b"quote-message-fixture")
    (bundle / "quote.sig").write_bytes(b"quote-signature-fixture")
    (bundle / "ak.pub").write_bytes(b"ak-public-fixture")
    (bundle / "ek.pub").write_bytes(b"ek-public-fixture")
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
            {"sequence": 1, "pcr": 4, "event_type": "EV_EFI_BOOT_SERVICES_APPLICATION", "digest_sha256": "11" * 32, "session_id": "self-test-session"},
            {"sequence": 2, "pcr": 4, "event_type": "EV_SEPARATOR", "digest_sha256": "22" * 32, "session_id": "self-test-session"},
            {"sequence": 3, "pcr": 7, "event_type": "EV_EFI_VARIABLE_AUTHORITY", "digest_sha256": "33" * 32, "session_id": "self-test-session"},
            {"sequence": 4, "pcr": 0, "event_type": "EV_ACTION", "digest_sha256": "44" * 32, "session_id": "self-test-session"},
            {"sequence": 5, "pcr": 2, "event_type": "EV_ACTION", "digest_sha256": "55" * 32, "session_id": "self-test-session"},
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
    raw_eventlog_path.write_text(
        json.dumps(
            {
                "parser_id": "mycelix.pc-client.raw-tpm2-eventlog-parser.v0.1",
                "parser_source_sha256": platform.sha256_file(
                    SECURITY / "parse_mycelix_raw_tpm2_eventlog_v0_1.py"
                ),
                "binary_sha256": platform.sha256_file(bundle / "eventlog.bin"),
                "events": [
                    {
                        "sequence": event["sequence"],
                        "pcr": event["pcr"],
                        "event_type": event["event_type"],
                        "digest_sha256": event.get("digest_sha256"),
                        "payload_hex": "",
                    }
                    for event in event_stream["events"]
                ],
            },
            indent=2,
            sort_keys=True,
        ) + "\n",
        encoding="utf-8",
    )

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
    manifest["artifacts"]["reconstruction_input_sha256"] = platform.sha256_file(input_path)
    manifest["artifacts"]["observed_pcr_values_file_sha256"] = platform.sha256_file(observed_pcr_path)
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
