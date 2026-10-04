#!/usr/bin/env python3
"""Executable TPM 2.0 Evidence qualifier for Mycelix v0.1.

The profile intentionally uses a software TPM so the Evidence chain can be
exercised without claiming hardware-root security. A missing dependency is
reported as NOT EXECUTED, never as PASS.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "docs" / "security" / "mycelix-tpm2-evidence-profile-v0.1.json"


def run(argv: Sequence[str], env: dict[str, str], cwd: Path, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(argv),
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=check,
    )


def require_tools() -> list[str]:
    names = [
        "swtpm",
        "tpm2_createek",
        "tpm2_createak",
        "tpm2_pcrread",
        "tpm2_pcrextend",
        "tpm2_quote",
        "tpm2_checkquote",
    ]
    return [name for name in names if shutil.which(name) is None]


def version_text(binary: str, env: dict[str, str], cwd: Path) -> str:
    proc = run([binary, "--version"], env, cwd, check=False)
    return (proc.stdout + proc.stderr).strip()


def extract_version(text: str) -> str | None:
    match = re.search(r"(?<![0-9])([0-9]+\.[0-9]+(?:\.[0-9]+)?)(?![0-9])", text)
    return match.group(1) if match else None


def pcr16_value(path: Path) -> bytes:
    text = path.read_text(encoding="utf-8")
    match = re.search(r"\b16:\s*([0-9a-fA-F]{64})\b", text)
    if not match:
        raise RuntimeError(f"could not parse SHA-256 PCR16 from {path}")
    return bytes.fromhex(match.group(1))


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def start_swtpm(state_dir: Path, socket_dir: Path, env: dict[str, str]) -> subprocess.Popen[str]:
    server = socket_dir / "swtpm.sock"
    control = socket_dir / "swtpm.sock.ctrl"
    log = socket_dir / "swtpm.log"

    cmd = [
        "swtpm",
        "socket",
        "--tpm2",
        "--tpmstate",
        f"dir={state_dir},mode=0600,lock,fsync",
        "--server",
        f"type=unixio,path={server},mode=0600",
        "--ctrl",
        f"type=unixio,path={control},mode=0600",
        "--flags",
        "not-need-init,startup-clear",
        "--log",
        f"file={log},level=20",
    ]
    proc = subprocess.Popen(
        cmd,
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    for _ in range(50):
        if server.exists():
            return proc
        if proc.poll() is not None:
            stderr = proc.stderr.read() if proc.stderr else ""
            raise RuntimeError(f"swtpm exited before socket creation: {stderr}")
        time.sleep(0.1)
    proc.terminate()
    raise RuntimeError("swtpm socket did not appear within 5 seconds")


def main() -> int:
    profile = json.loads(PROFILE.read_text(encoding="utf-8"))
    missing = require_tools()
    if missing:
        print("TPM2 QUALIFICATION: NOT EXECUTED")
        print("Missing required tools: " + ", ".join(missing))
        return 2

    with tempfile.TemporaryDirectory(prefix="mycelix-tpm2-v0.1-") as tmp:
        root = Path(tmp)
        state = root / "state"
        sockets = root / "sockets"
        capture = root / "capture"
        state.mkdir()
        sockets.mkdir()
        capture.mkdir()

        env = os.environ.copy()
        env["TPM2TOOLS_TCTI"] = f"swtpm:path={sockets / 'swtpm.sock'}"

        swtpm_proc: subprocess.Popen[str] | None = None
        try:
            swtpm_proc = start_swtpm(state, sockets, env)

            swtpm_version_text = version_text("swtpm", env, root)
            tool_versions: dict[str, str | None] = {"swtpm": extract_version(swtpm_version_text)}
            for tool in [
                "tpm2_createek",
                "tpm2_createak",
                "tpm2_pcrread",
                "tpm2_pcrextend",
                "tpm2_quote",
                "tpm2_checkquote",
            ]:
                tool_versions[tool] = extract_version(version_text(tool, env, root))

            expected_swtpm = profile["selected_implementation"]["tpm"]["version"]
            expected_tpm2 = profile["selected_implementation"]["tooling"]["tpm2_tools"]
            observed_tpm2_versions = {v for k, v in tool_versions.items() if k.startswith("tpm2_") and v}
            if tool_versions["swtpm"] != expected_swtpm:
                raise RuntimeError(f"swtpm version mismatch: expected {expected_swtpm}, got {tool_versions['swtpm']}")
            if observed_tpm2_versions != {expected_tpm2}:
                raise RuntimeError(f"tpm2-tools version mismatch: expected {expected_tpm2}, got {sorted(observed_tpm2_versions)}")

            run(["tpm2_createek", "-Q", "-c", str(root / "ek.ctx"), "-G", "rsa", "-u", str(root / "ek.pub")], env, root)
            run(
                [
                    "tpm2_createak",
                    "-Q",
                    "-C",
                    str(root / "ek.ctx"),
                    "-c",
                    str(root / "ak.ctx"),
                    "-G",
                    "rsa",
                    "-g",
                    "sha256",
                    "-s",
                    "rsassa",
                    "-u",
                    str(root / "ak.pub"),
                    "-n",
                    str(root / "ak.name"),
                ],
                env,
                root,
            )

            before_path = capture / "pcr-before.yaml"
            run(["tpm2_pcrread", "sha256:16"], env, root, check=True).check_returncode
            before = run(["tpm2_pcrread", "sha256:16"], env, root)
            before_path.write_text(before.stdout, encoding="utf-8")
            before_pcr = pcr16_value(before_path)

            manifest = (
                b'{"profile":"mycelix-tpm2-evidence-vptm","workload":"qualification-fixture",'
                b'"version":"0.1.0","purpose":"PCR-binding-test"}\n'
            )
            manifest_path = capture / "workload-manifest.json"
            manifest_path.write_bytes(manifest)
            workload_digest = hashlib.sha256(manifest).digest()
            workload_hex = workload_digest.hex()

            run(["tpm2_pcrextend", f"16:sha256={workload_hex}"], env, root)
            after = run(["tpm2_pcrread", "sha256:16"], env, root)
            after_path = capture / "pcr-after.yaml"
            after_path.write_text(after.stdout, encoding="utf-8")
            after_pcr = pcr16_value(after_path)

            expected_after = hashlib.sha256(before_pcr + workload_digest).digest()
            if after_pcr != expected_after:
                raise RuntimeError(
                    "PCR16 reconstruction mismatch: TPM PCR does not equal SHA256(previous_PCR16 || workload_digest)"
                )

            nonce = hashlib.sha256(
                b"mycelix-rats-tpm2-v0.1:deterministic-qualification-nonce"
            ).digest()

            run(
                [
                    "tpm2_quote",
                    "-Q",
                    "-c",
                    str(root / "ak.ctx"),
                    "-l",
                    "sha256:16",
                    "-q",
                    nonce.hex(),
                    "-m",
                    str(capture / "quote.msg"),
                    "-s",
                    str(capture / "quote.sig"),
                    "-o",
                    str(capture / "quote.pcrs"),
                    "-g",
                    "sha256",
                ],
                env,
                root,
            )

            canonical = run(
                [
                    "tpm2_checkquote",
                    "-u",
                    str(root / "ak.pub"),
                    "-m",
                    str(capture / "quote.msg"),
                    "-s",
                    str(capture / "quote.sig"),
                    "-f",
                    str(capture / "quote.pcrs"),
                    "-g",
                    "sha256",
                    "-q",
                    nonce.hex(),
                    "-l",
                    "sha256:16",
                ],
                env,
                root,
            )

            wrong_nonce = bytes.fromhex("00" * 32)
            negative_nonce = run(
                [
                    "tpm2_checkquote",
                    "-u",
                    str(root / "ak.pub"),
                    "-m",
                    str(capture / "quote.msg"),
                    "-s",
                    str(capture / "quote.sig"),
                    "-f",
                    str(capture / "quote.pcrs"),
                    "-g",
                    "sha256",
                    "-q",
                    wrong_nonce.hex(),
                    "-l",
                    "sha256:16",
                ],
                env,
                root,
                check=False,
            )
            if negative_nonce.returncode == 0:
                raise RuntimeError("wrong nonce unexpectedly verified")

            tampered_pcrs = capture / "quote.pcrs.tampered"
            tampered = bytearray((capture / "quote.pcrs").read_bytes())
            if not tampered:
                raise RuntimeError("quote PCR output is empty")
            tampered[-1] ^= 0x01
            tampered_pcrs.write_bytes(tampered)

            negative_pcr = run(
                [
                    "tpm2_checkquote",
                    "-u",
                    str(root / "ak.pub"),
                    "-m",
                    str(capture / "quote.msg"),
                    "-s",
                    str(capture / "quote.sig"),
                    "-f",
                    str(tampered_pcrs),
                    "-g",
                    "sha256",
                    "-q",
                    nonce.hex(),
                    "-l",
                    "sha256:16",
                ],
                env,
                root,
                check=False,
            )
            if negative_pcr.returncode == 0:
                raise RuntimeError("tampered PCR material unexpectedly verified")

            evidence = {
                "profile_id": profile["profile_id"],
                "profile_version": profile["profile_version"],
                "claim_ceiling": profile["claim_ceiling"],
                "tpm_implementation": "swtpm",
                "swtpm_version": tool_versions["swtpm"],
                "tpm2_tools_version": expected_tpm2,
                "transport": "local-unix-domain-socket",
                "pcr_selection": "sha256:16",
                "attestation_key_algorithm": "RSA-2048",
                "signing_scheme": "RSASSA-SHA256",
                "nonce_sha256": sha256_hex(nonce),
                "workload_manifest_sha256": sha256_hex(manifest),
                "pcr16_before": before_pcr.hex(),
                "pcr16_after": after_pcr.hex(),
                "expected_pcr16_after": expected_after.hex(),
                "quote_message_sha256": sha256_hex((capture / "quote.msg").read_bytes()),
                "quote_signature_sha256": sha256_hex((capture / "quote.sig").read_bytes()),
                "quote_pcr_output_sha256": sha256_hex((capture / "quote.pcrs").read_bytes()),
                "canonical_quote_verification": canonical.returncode == 0,
                "wrong_nonce_rejected": negative_nonce.returncode != 0,
                "tampered_pcr_rejected": negative_pcr.returncode != 0,
                "platform_measured_boot_established": False,
                "physical_tpm_security_established": False,
                "manufacturer_ek_trust_established": False,
                "local_authorization_established": False,
            }
            (capture / "mycelix-tpm2-evidence-envelope-v0.1.json").write_text(
                json.dumps(evidence, indent=2) + "
",
                encoding="utf-8",
            )

            print("TPM2 QUALIFICATION: PASS")
            print("Canonical quote verification: PASS")
            print("Wrong nonce rejection: PASS")
            print("Tampered PCR rejection: PASS")
            print("PCR16 independent reconstruction: PASS")
            print("Claim ceiling: ReferenceModelOnly")
            return 0
        except Exception as exc:
            print(f"TPM2 QUALIFICATION: FAIL: {exc}")
            return 1
        finally:
            if swtpm_proc is not None:
                swtpm_proc.send_signal(signal.SIGTERM)
                try:
                    swtpm_proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    swtpm_proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
