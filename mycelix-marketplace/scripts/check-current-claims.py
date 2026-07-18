#!/usr/bin/env python3
"""Keep canonical entry points aligned with the current evidence ledger."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
files = [ROOT / "README.md", ROOT / "backend/README.md", ROOT / "backend/happ.yaml"]
forbidden = ["45% Byzantine", "45% BFT", "production-ready", "Production Ready"]
errors: list[str] = []
for path in files:
    text = path.read_text()
    for phrase in forbidden:
        if phrase in text:
            errors.append(f"{path.relative_to(ROOT)} contains forbidden current claim: {phrase}")

ledger = (ROOT / "docs/CLAIMS_AND_EVIDENCE.md").read_text()
for required in ["Do not claim as demonstrated", "Pre-alpha", "Historical completion reports"]:
    if required not in ledger:
        errors.append(f"claims ledger missing required boundary: {required}")

if errors:
    raise SystemExit("Current-claims check failed:\n- " + "\n- ".join(errors))
print("canonical Marketplace claims match the evidence ledger")
