#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import tarfile
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bundle", type=Path)
    args = parser.parse_args()
    bundle_path = args.bundle.resolve()
    if not bundle_path.is_file():
        raise SystemExit(f"bundle not found: {bundle_path}")

    with tempfile.TemporaryDirectory() as raw:
        temp = Path(raw)
        with tarfile.open(bundle_path, "r:gz") as archive:
            for member in archive.getmembers():
                parts = Path(member.name).parts
                if member.name.startswith("/") or ".." in parts:
                    raise SystemExit(f"unsafe archive path: {member.name}")
                if not (member.isdir() or member.isreg()):
                    raise SystemExit(f"archive links/devices are forbidden: {member.name}")
            archive.extractall(temp, filter="data")
        root = temp / "mycelix-promotion-v1"
        manifest_path = root / "release-manifest.json"
        signature = root / "release-manifest.sig"
        public_key = root / "release-public-key.pem"
        for path in [manifest_path, signature, public_key]:
            if not path.is_file():
                raise SystemExit(f"release envelope is incomplete: {path.name}")
        subprocess.run(
            ["openssl", "pkeyutl", "-verify", "-rawin", "-pubin", "-inkey", str(public_key), "-in", str(manifest_path), "-sigfile", str(signature)],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        manifest = json.loads(manifest_path.read_text())
        if manifest.get("schema_version") != 1:
            raise SystemExit("unsupported release manifest schema")
        if manifest.get("public_key_sha256") != sha(public_key):
            raise SystemExit("release public key digest mismatch")
        expected = {"release-manifest.json", "release-manifest.sig", "release-public-key.pem"}
        for item in manifest.get("payload", []):
            relative = item.get("path")
            if not isinstance(relative, str) or relative.startswith("/") or ".." in Path(relative).parts:
                raise SystemExit("unsafe payload path in release manifest")
            path = root / relative
            if not path.is_file() or path.is_symlink():
                raise SystemExit(f"payload file missing: {relative}")
            if path.stat().st_size != item.get("size") or sha(path) != item.get("sha256"):
                raise SystemExit(f"payload digest mismatch: {relative}")
            expected.add(relative)
        actual = {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file()}
        if actual != expected:
            raise SystemExit(f"unmanifested release files: {sorted(actual - expected)}")

        artifacts = json.loads((root / "receipts/artifact-manifest.json").read_text())
        if manifest.get("artifact_manifest_sha256") != sha(root / "receipts/artifact-manifest.json"):
            raise SystemExit("artifact manifest digest mismatch")
        if artifacts.get("source_revision") != manifest.get("source_revision") or artifacts.get("profile") != manifest.get("profile"):
            raise SystemExit("release and artifact identity mismatch")
        command = [
            "python3", str(ROOT / "scripts/verify-live-evidence.py"), str(root / "evidence"),
            "--profile", manifest["profile"], "--source-revision", manifest["source_revision"],
            "--happ", str(root / "artifacts/mycelix_marketplace.happ"),
            "--marketplace-dna", str(root / "artifacts/mycelix_marketplace.dna"),
            "--output", str(temp / "promotion-reverified.json"),
        ]
        if manifest["profile"] == "settlement":
            command += ["--finance-dna", str(root / "artifacts/finance.dna")]
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
        reverified = json.loads((temp / "promotion-reverified.json").read_text())
        if reverified.get("result") != manifest.get("promotion_result"):
            raise SystemExit("promotion result changed during bundle verification")
    print(f"verified signed promotion bundle: {bundle_path}")


if __name__ == "__main__":
    main()
