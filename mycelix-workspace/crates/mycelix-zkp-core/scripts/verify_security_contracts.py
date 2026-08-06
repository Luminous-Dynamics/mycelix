#!/usr/bin/env python3
"""Zero-dependency source contract checks for mycelix-zkp-core.

These checks are not a Rust compiler or cryptographic test suite. They prevent
specific security regressions from silently reappearing when full toolchains are
unavailable or before expensive feature-matrix CI begins.
"""

from __future__ import annotations

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
FAILURES: list[str] = []


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        FAILURES.append(message)


def balanced_rust_delimiters(text: str, path: str) -> None:
    """Lightweight delimiter check after removing comments and strings."""
    scrubbed = re.sub(r'//.*', '', text)
    scrubbed = re.sub(r'/\*.*?\*/', '', scrubbed, flags=re.S)
    scrubbed = re.sub(r'"(?:\\.|[^"\\])*"', '""', scrubbed)
    pairs = {'}': '{', ']': '[', ')': '('}
    stack: list[str] = []
    for char in scrubbed:
        if char in '{[(':
            stack.append(char)
        elif char in '}])':
            if not stack or stack.pop() != pairs[char]:
                FAILURES.append(f"{path}: unbalanced delimiter {char!r}")
                return
    if stack:
        FAILURES.append(f"{path}: unclosed delimiters {stack[-10:]}")


def extract_struct_initializers(text: str, type_name: str) -> list[str]:
    """Extract simple Rust struct literal bodies using brace depth."""
    results: list[str] = []
    pattern = re.compile(rf'\b{re.escape(type_name)}\s*\{{')
    for match in pattern.finditer(text):
        # Skip the actual struct declaration and impl block.
        prefix = text[max(0, match.start() - 16):match.start()]
        if re.search(r'\b(struct|impl)\s*$', prefix):
            continue
        start = text.find('{', match.start())
        depth = 0
        for idx in range(start, len(text)):
            if text[idx] == '{':
                depth += 1
            elif text[idx] == '}':
                depth -= 1
                if depth == 0:
                    results.append(text[start + 1:idx])
                    break
    return results


types = read("src/types.rs")
validation = read("src/validation.rs")
backend = read("src/backend.rs")
dilithium = read("src/dilithium.rs")
merkle = read("src/circuits/merkle_membership.rs")
nullifier = read("src/circuits/nullifier.rs")
supply = read("src/supply.rs")
records = read("src/supply_verification.rs")
cargo = read("Cargo.toml")
capabilities = read("CAPABILITY_MATRIX.md")

require("pub const AUTHENTICATED_PROOF_PROTOCOL_VERSION: u32 = 2" in types,
        "authenticated proof protocol must remain v2")
require("energy_millijoules: u64" in types,
        "energy must use a canonical integer field")
require("joules_consumed" not in "\n".join(
    p.read_text(encoding="utf-8") for p in ROOT.rglob("*.rs")
), "legacy unsigned floating-point joules_consumed field reintroduced")
for bound_fragment in (
    "self.metadata.backend.wire_id()",
    "self.energy_millijoules.to_le_bytes()",
    "self.public_inputs_hash",
    "Sha256::digest(&self.proof)",
    "self.metadata.nonce",
):
    require(bound_fragment in types, f"signed transcript missing {bound_fragment}")

require("validate_authenticated_proof_envelope" in validation,
        "fail-closed envelope validator missing")
for check in (
    "proof.metadata.protocol_version != policy.protocol_version",
    "proof.metadata.nonce == [0; 32]",
    "proof.public_inputs_hash == [0; 32]",
    "proof.metadata.timestamp > latest_allowed",
    "proof.metadata.timestamp < oldest_allowed",
    "policy.allowed_backends.contains",
):
    require(check in validation, f"envelope validator missing check: {check}")

require("BackendId::Miden" in types, "Miden backend identifier missing")
require("false // Structural adapter only" in backend,
        "RISC Zero adapter must not report operational availability")
require("pub fn select_backend(complexity: CircuitComplexity) -> Option<BackendId>" in backend,
        "backend selector must return None instead of an unavailable default")
require("backend-risc0 = []" in cargo and "Structural adapter only" in cargo,
        "Cargo feature must disclose structural-only RISC Zero status")

require("validate_membership_proof_structure" in merkle,
        "Merkle structural validator must be explicitly named")
require("validate_nullifier_proof_structure" in nullifier,
        "nullifier structural validator must be explicitly named")
require("does not itself implement a zero-knowledge" in merkle,
        "Merkle module must not imply structural validation is ZK verification")
require("does not itself verify zero-knowledge membership" in nullifier,
        "nullifier module must not imply structural validation is ZK verification")

for field in (
    "subject_id",
    "policy_hash",
    "evidence_commitment",
    "public_values_hash",
    "issuer_id",
    "valid_from",
    "valid_until",
    "nonce",
):
    require(f"self.{field}" in supply, f"supply commitment does not bind {field}")
require("proof.metadata.domain_tag != expected_domain" in supply,
        "supply envelope must enforce exact proof-kind domain")
require("proof.public_inputs_hash != statement.commitment()" in supply,
        "supply envelope must bind statement commitment")

for field in (
    "statement_commitment",
    "proof_hash",
    "verifier_id",
    "verifier_implementation",
    "verifier_version",
    "verification_policy_hash",
    "backend",
    "status",
    "verified_at",
):
    require(f"self.{field}" in records, f"verification record digest does not bind {field}")
require("Valid" in records and "Invalid" in records and "Unsupported" in records and "Expired" in records,
        "verification records need explicit non-boolean outcomes")

require("RISC Zero backend | Structural adapter only" in capabilities,
        "capability matrix must disclose RISC Zero status")
require("Merkle membership envelope | Implemented | Format validation" in capabilities,
        "capability matrix must distinguish Merkle structure from proof verification")

# All AuthenticatedProof literals must initialize the canonical energy field.
for path in list((ROOT / "src").rglob("*.rs")) + list((ROOT / "benches").rglob("*.rs")):
    text = path.read_text(encoding="utf-8")
    balanced_rust_delimiters(text, str(path.relative_to(ROOT)))
    for body in extract_struct_initializers(text, "AuthenticatedProof"):
        require("energy_millijoules:" in body,
                f"{path.relative_to(ROOT)}: AuthenticatedProof literal missing energy_millijoules")

if FAILURES:
    print("security contract verification FAILED", file=sys.stderr)
    for failure in FAILURES:
        print(f" - {failure}", file=sys.stderr)
    raise SystemExit(1)

print("security contract verification passed")
