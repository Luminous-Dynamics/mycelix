#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN_NAMES = {"actors.env", ".env", "id_ed25519", "private.pem", "secret.key"}
FORBIDDEN_TEXT = (b"TOKEN_BASE64=", b"BEGIN PRIVATE KEY", b"BEGIN OPENSSH PRIVATE KEY")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def safe_payload(path: Path, relative: Path) -> None:
    if path.is_symlink() or not path.is_file():
        raise SystemExit(f"bundle payload must be a regular file: {relative}")
    lowered = relative.name.lower()
    if lowered in FORBIDDEN_NAMES or any(word in lowered for word in ["token", "private-key", "secret-key"]):
        raise SystemExit(f"sensitive-looking file cannot enter release evidence: {relative}")
    if path.stat().st_size <= 10_000_000:
        data = path.read_bytes()
        if any(marker in data for marker in FORBIDDEN_TEXT):
            raise SystemExit(f"credential material detected in release evidence: {relative}")


def copy_file(source: Path, destination: Path) -> None:
    if not source.is_file() or source.is_symlink():
        raise SystemExit(f"required release file is not a regular file: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=["base", "settlement", "arbitration", "network"], required=True)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--private-key", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    artifact_root = args.artifact_root.resolve()
    evidence = artifact_root / f"evidence-{args.profile}"
    artifact_manifest_path = artifact_root / "artifact-manifest.json"
    if not evidence.is_dir() or not artifact_manifest_path.is_file():
        raise SystemExit("artifact root must contain artifact-manifest.json and profile evidence")
    artifact_manifest = json.loads(artifact_manifest_path.read_text())
    if artifact_manifest.get("profile") != args.profile:
        raise SystemExit("artifact profile does not match requested release profile")
    private_key = args.private_key.resolve()
    if not private_key.is_file():
        raise SystemExit(f"release signing key is missing: {private_key}")

    out = args.output_dir.resolve()
    if out.exists() and any(out.iterdir()):
        raise SystemExit(f"output directory must be empty: {out}")
    out.mkdir(parents=True, exist_ok=True)
    bundle = out / "mycelix-promotion-v1"

    copy_file(artifact_manifest_path, bundle / "receipts/artifact-manifest.json")
    for name in ["toolchain.json", "zome-artifacts.json", "SHA256SUMS"]:
        copy_file(artifact_root / name, bundle / f"receipts/{name}")
    for name in ["mycelix_marketplace.happ", "mycelix_marketplace.dna"]:
        copy_file(artifact_root / "artifacts" / name, bundle / "artifacts" / name)
    finance = artifact_manifest.get("artifacts", {}).get("finance_dna")
    if finance:
        copy_file(Path(finance["path"]), bundle / "artifacts/finance.dna")
    for child in sorted(evidence.iterdir()):
        if child.is_file():
            copy_file(child, bundle / "evidence" / child.name)

    public_key = bundle / "release-public-key.pem"
    subprocess.run(
        ["openssl", "pkey", "-in", str(private_key), "-pubout", "-out", str(public_key)],
        check=True,
        stdout=subprocess.DEVNULL,
    )
    payload = []
    for path in sorted(bundle.rglob("*")):
        if not path.is_file() or path == public_key:
            continue
        relative = path.relative_to(bundle)
        safe_payload(path, relative)
        payload.append({"path": relative.as_posix(), "sha256": sha(path), "size": path.stat().st_size})

    promotion = json.loads((bundle / "evidence/promotion.json").read_text())
    manifest = {
        "schema_version": 1,
        "profile": args.profile,
        "source_revision": artifact_manifest["source_revision"],
        "promotion_result": promotion.get("result"),
        "public_key_sha256": sha(public_key),
        "artifact_manifest_sha256": sha(bundle / "receipts/artifact-manifest.json"),
        "payload": payload,
    }
    manifest_path = bundle / "release-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    signature = bundle / "release-manifest.sig"
    subprocess.run(
        ["openssl", "pkeyutl", "-sign", "-rawin", "-inkey", str(private_key), "-in", str(manifest_path), "-out", str(signature)],
        check=True,
    )
    os.chmod(signature, 0o644)
    print(bundle)


if __name__ == "__main__":
    main()
