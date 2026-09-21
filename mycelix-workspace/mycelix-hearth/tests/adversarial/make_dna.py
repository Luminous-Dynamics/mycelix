#!/usr/bin/env python3
"""Generate a qualification-only Hearth DNA with appended adversarial zomes.

The canonical production manifest is the source of truth. This script changes
only the DNA name/network seed, relative WASM paths required by the deeper
temporary workdir, and appends test-only zomes. It never edits production
`dna/dna.yaml`.
"""
from pathlib import Path
import shutil

TESTS = Path(__file__).resolve().parents[1]
ROOT = TESTS.parent
CANONICAL = ROOT / "dna" / "dna.yaml"
WORKDIR = TESTS / ".generated-care-transition-adversarial-dna"
MANIFEST = WORKDIR / "dna.yaml"


def require_once(text: str, needle: str, label: str) -> None:
    count = text.count(needle)
    if count != 1:
        raise SystemExit(f"expected exactly one {label}, found {count}")


text = CANONICAL.read_text(encoding="utf-8")
for forbidden in (
    "hearth_care_adversarial_fake_integrity",
    "hearth_care_adversarial_fake",
    "hearth_care_adversarial_raw_completion",
):
    if forbidden in text:
        raise SystemExit(f"qualification fixture leaked into canonical DNA: {forbidden}")

require_once(text, "name: mycelix_hearth\n", "canonical DNA name")
require_once(text, 'network_seed: "mycelix-hearth-v1"', "canonical network seed")
require_once(text, "\ncoordinator:\n", "coordinator section")
require_once(
    text,
    "    - name: hearth_care_transitions_integrity\n",
    "canonical Care transition integrity tail",
)
require_once(
    text,
    "    - name: hearth_care_transitions\n",
    "canonical Care transition coordinator tail",
)

text = text.replace(
    "name: mycelix_hearth\n",
    "name: mycelix_hearth_care_adversarial\n",
    1,
)
text = text.replace(
    'network_seed: "mycelix-hearth-v1"',
    'network_seed: "mycelix-hearth-v1-adversarial-ci"',
    1,
)

# The generated manifest lives at tests/<workdir>/dna.yaml. Canonical zome
# outputs are under Hearth root target/, two directory levels above it.
text = text.replace("path: ../target/", "path: ../../target/")

marker = "\ncoordinator:\n"
fake_integrity = """
    # QUALIFICATION ONLY: Serde-lookalike attack fixture. Never ship.
    - name: hearth_care_adversarial_fake_integrity
      path: ../adversarial/target/wasm32-unknown-unknown/release/hearth_care_adversarial_fake_integrity.wasm
"""
text = text.replace(marker, fake_integrity + marker, 1)

fixture_coordinators = """

    # QUALIFICATION ONLY: authors entries under the fake integrity AppEntryDef.
    - name: hearth_care_adversarial_fake
      path: ../adversarial/target/wasm32-unknown-unknown/release/hearth_care_adversarial_fake.wasm
      dependencies:
        - name: hearth_care_adversarial_fake_integrity

    # QUALIFICATION ONLY: submits raw payloads to the real transition validator.
    - name: hearth_care_adversarial_raw_completion
      path: ../adversarial/target/wasm32-unknown-unknown/release/hearth_care_adversarial_raw_completion.wasm
      dependencies:
        - name: hearth_care_transitions_integrity
"""
text = text.rstrip() + fixture_coordinators + "\n"

if WORKDIR.exists():
    shutil.rmtree(WORKDIR)
WORKDIR.mkdir(parents=True)
MANIFEST.write_text(text, encoding="utf-8")
print(MANIFEST)
