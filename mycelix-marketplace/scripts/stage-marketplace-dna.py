#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = json.loads((ROOT / "contracts/promotion-artifacts-v1.json").read_text())


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-dir", type=Path, required=True)
    parser.add_argument("--stage-dir", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()

    target = args.target_dir.resolve()
    stage = args.stage_dir.resolve()
    if stage.exists():
        shutil.rmtree(stage)
    stage.mkdir(parents=True)
    shutil.copy2(ROOT / SPEC["marketplace"]["dna_manifest"], stage / "dna.yaml")

    records = []
    for mapping in SPEC["marketplace"]["zomes"]:
        source = target / mapping["rust_artifact"]
        if not source.is_file():
            raise SystemExit(f"missing Rust WASM artifact: {source}")
        destination = stage / mapping["dna_path"]
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        records.append({
            "dna_path": mapping["dna_path"],
            "rust_artifact": mapping["rust_artifact"],
            "sha256": sha256(destination),
            "size": destination.stat().st_size,
        })

    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps({"schema_version": 1, "zomes": records}, indent=2) + "\n")
    print(stage)


if __name__ == "__main__":
    main()
