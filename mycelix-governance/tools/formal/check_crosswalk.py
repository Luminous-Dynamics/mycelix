#!/usr/bin/env python3
"""Fail-closed census/ownership drift gate for Rust ↔ Alloy/TLA+.

This is deliberately narrower than a behavioral refinement proof. It verifies
that the constitutional vocabulary and declared abstraction boundary remain
explicitly synchronized before formal model-check evidence is used to justify
claims about the Rust implementation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
DEFAULT_CROSSWALK = REPO / "mycelix-governance" / "specs" / "constitutional-formal-crosswalk.json"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//.*", "", text)
    return text


def braced_body(text: str, marker: str) -> str:
    start = text.find(marker)
    if start < 0:
        raise ValueError(f"marker not found: {marker}")
    open_i = text.find("{", start + len(marker))
    if open_i < 0:
        raise ValueError(f"opening brace not found after: {marker}")
    depth = 0
    for i in range(open_i, len(text)):
        ch = text[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[open_i + 1 : i]
    raise ValueError(f"unterminated braced body: {marker}")


def rust_unit_enum(text: str, name: str) -> list[str]:
    body = strip_comments(braced_body(text, f"pub enum {name}"))
    variants = re.findall(r"(?m)^\s*([A-Z][A-Za-z0-9_]*)\s*,\s*$", body)
    if not variants:
        raise ValueError(f"no unit variants parsed for Rust enum {name}")
    return variants


def rust_type_exists(text: str, name: str) -> bool:
    return bool(re.search(rf"\bpub\s+(?:enum|struct|type)\s+{re.escape(name)}\b", text))


def rust_field_exists(text: str, struct_name: str, field: str) -> bool:
    try:
        body = strip_comments(braced_body(text, f"pub struct {struct_name}"))
    except ValueError:
        return False
    return bool(re.search(rf"(?m)^\s*pub\s+{re.escape(field)}\s*:", body))


def alloy_enum(text: str, name: str) -> list[str]:
    body = strip_comments(braced_body(text, f"enum {name}"))
    items = [part.strip() for part in body.split(",")]
    values = [item for item in items if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", item)]
    if not values:
        raise ValueError(f"no variants parsed for Alloy enum {name}")
    return values


def alloy_ownership(text: str, powers: set[str]) -> dict[str, str]:
    ownership: dict[str, str] = {}
    pattern = re.compile(
        r"\(a\.power\s+(?:in|=)\s+(.*?)\)\s*iff\s+a\.owner\s*=\s*([A-Za-z_][A-Za-z0-9_]*)",
        re.S,
    )
    for expr, owner in pattern.findall(text):
        mentioned = [tok for tok in re.findall(r"\b[A-Z][A-Za-z0-9_]*\b", expr) if tok in powers]
        if not mentioned:
            raise ValueError(f"ownership clause for {owner} contains no known SovereignPower")
        for power in mentioned:
            if power in ownership:
                raise ValueError(f"Alloy ownership map assigns {power} more than once")
            ownership[power] = owner
    return ownership


def tla_constants(text: str) -> list[str]:
    match = re.search(r"(?m)^CONSTANTS\s+([^\n]+)$", text)
    if not match:
        raise ValueError("TLA+ CONSTANTS declaration not found")
    return re.findall(r"\b[A-Za-z_][A-Za-z0-9_]*\b", match.group(1))


def tla_variables(text: str) -> list[str]:
    match = re.search(r"(?ms)^VARIABLES\s+(.*?)\n\s*vars\s*==", text)
    if not match:
        raise ValueError("TLA+ VARIABLES declaration not found")
    return re.findall(r"\b[A-Za-z_][A-Za-z0-9_]*\b", match.group(1))


def tla_definition_exists(text: str, name: str) -> bool:
    return bool(re.search(rf"(?m)^\s*{re.escape(name)}\s*==", text))


def sorted_set(values: Any) -> list[str]:
    return sorted(set(values))


def compare_sets(label: str, actual: set[str], expected: set[str], errors: list[str]) -> None:
    if actual == expected:
        return
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    errors.append(f"{label} mismatch: missing={missing}, extra={extra}")


def check_rust_symbol(symbol: str, sources: list[str]) -> bool:
    if "." not in symbol:
        return any(rust_type_exists(text, symbol) for text in sources)
    type_name, field = symbol.split(".", 1)
    return any(rust_field_exists(text, type_name, field) for text in sources)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--crosswalk", type=Path, default=DEFAULT_CROSSWALK)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    crosswalk_path = args.crosswalk.resolve()
    crosswalk = json.loads(crosswalk_path.read_text(encoding="utf-8"))

    authority_path = REPO / crosswalk["rust"]["authority_source"]
    consumption_path = REPO / crosswalk["rust"]["consumption_source"]
    alloy_path = REPO / crosswalk["formal"]["alloy_authority"]
    tla_path = REPO / crosswalk["formal"]["tla_consumption"]

    authority = authority_path.read_text(encoding="utf-8")
    consumption = consumption_path.read_text(encoding="utf-8")
    alloy = alloy_path.read_text(encoding="utf-8")
    tla = tla_path.read_text(encoding="utf-8")
    rust_sources = [authority, consumption]

    errors: list[str] = []

    rust_branches = set(rust_unit_enum(authority, "Branch"))
    rust_guardians = set(rust_unit_enum(authority, "Guardian"))
    rust_powers = set(rust_unit_enum(authority, "ConstitutionalPower"))
    rust_entitlements = set(rust_unit_enum(authority, "ConstitutionalEntitlement"))
    rust_cutoffs = set(rust_unit_enum(consumption, "RevocationCutoff"))

    alloy_principals = set(alloy_enum(alloy, "PrincipalClass"))
    alloy_powers = set(alloy_enum(alloy, "SovereignPower"))
    alloy_entitlements = set(alloy_enum(alloy, "Entitlement"))
    ownership = alloy_ownership(alloy, alloy_powers)

    crosswalk_powers: dict[str, str] = crosswalk["powers"]
    crosswalk_entitlements = set(crosswalk["entitlements"])
    branch_map: dict[str, str] = crosswalk["principal_classes"]["branches"]
    guardian_map: dict[str, str] = crosswalk["principal_classes"]["guardians"]
    special_map: dict[str, str] = crosswalk["principal_classes"]["special"]

    compare_sets("Rust powers vs crosswalk", rust_powers, set(crosswalk_powers), errors)
    compare_sets("Alloy powers vs crosswalk", alloy_powers, set(crosswalk_powers), errors)
    compare_sets("Rust entitlements vs crosswalk", rust_entitlements, crosswalk_entitlements, errors)
    compare_sets("Alloy entitlements vs crosswalk", alloy_entitlements, crosswalk_entitlements, errors)
    compare_sets("Rust branches vs crosswalk", rust_branches, set(branch_map), errors)
    compare_sets("Rust guardians vs crosswalk", rust_guardians, set(guardian_map), errors)

    expected_principals = set(branch_map.values()) | set(guardian_map.values()) | set(special_map.values())
    compare_sets("Alloy PrincipalClass vs crosswalk mappings", alloy_principals, expected_principals, errors)

    unmapped_branch_values = set(branch_map.values()) - alloy_principals
    unmapped_guardian_values = set(guardian_map.values()) - alloy_principals
    if unmapped_branch_values:
        errors.append(f"branch mappings missing from Alloy PrincipalClass: {sorted(unmapped_branch_values)}")
    if unmapped_guardian_values:
        errors.append(f"guardian mappings missing from Alloy PrincipalClass: {sorted(unmapped_guardian_values)}")

    compare_sets("Alloy ownership keys vs powers", set(ownership), alloy_powers, errors)
    for power, expected_owner in sorted(crosswalk_powers.items()):
        actual_owner = ownership.get(power)
        if actual_owner != expected_owner:
            errors.append(f"owner mismatch for {power}: crosswalk={expected_owner}, alloy={actual_owner}")

    tla_spec = crosswalk["tla"]
    compare_sets("TLA+ constants", set(tla_constants(tla)), set(tla_spec["constants"]), errors)
    compare_sets("TLA+ variables", set(tla_variables(tla)), set(tla_spec["variables"]), errors)
    for invariant in tla_spec["required_invariants"]:
        if not tla_definition_exists(tla, invariant):
            errors.append(f"required TLA+ invariant/definition missing: {invariant}")

    cutoff_map: dict[str, str] = tla_spec["cutoff_values"]
    compare_sets("Rust RevocationCutoff vs crosswalk", rust_cutoffs, set(cutoff_map), errors)
    tla_string_literals = set(re.findall(r'"([A-Za-z_][A-Za-z0-9_]*)"', tla))
    missing_cutoff_values = set(cutoff_map.values()) - tla_string_literals
    if missing_cutoff_values:
        errors.append(f"TLA+ cutoff literals missing: {sorted(missing_cutoff_values)}")

    for mapping in tla_spec["abstractions"]:
        rust_symbol = mapping["rust_symbol"]
        tla_symbol = mapping["tla_symbol"]
        if not check_rust_symbol(rust_symbol, rust_sources):
            errors.append(f"crosswalk Rust abstraction symbol missing: {rust_symbol}")
        if tla_symbol not in set(tla_spec["variables"]) | set(tla_spec["constants"]):
            errors.append(f"crosswalk TLA+ abstraction symbol not declared in manifest: {tla_symbol}")

    for omission in tla_spec["out_of_model"]:
        rust_symbol = omission["rust_symbol"]
        if not check_rust_symbol(rust_symbol, rust_sources):
            errors.append(f"out_of_model Rust symbol no longer exists: {rust_symbol}")
        if not str(omission.get("reason", "")).strip():
            errors.append(f"out_of_model entry lacks reason: {rust_symbol}")

    inputs = [crosswalk_path, authority_path, consumption_path, alloy_path, tla_path, SCRIPT]
    receipt: dict[str, Any] = {
        "schema": "mycelix.constitutional-formal-drift-receipt.v1",
        "passed": not errors,
        "input_sha256": {str(p.relative_to(REPO)): sha256_file(p) for p in inputs},
        "census": {
            "rust": {
                "branches": sorted_set(rust_branches),
                "guardians": sorted_set(rust_guardians),
                "powers": sorted_set(rust_powers),
                "entitlements": sorted_set(rust_entitlements),
                "revocation_cutoffs": sorted_set(rust_cutoffs),
            },
            "alloy": {
                "principal_classes": sorted_set(alloy_principals),
                "powers": sorted_set(alloy_powers),
                "entitlements": sorted_set(alloy_entitlements),
                "ownership": dict(sorted(ownership.items())),
            },
            "crosswalk": {
                "power_count": len(crosswalk_powers),
                "entitlement_count": len(crosswalk_entitlements),
                "tla_constant_count": len(tla_spec["constants"]),
                "tla_variable_count": len(tla_spec["variables"]),
                "tla_invariant_count": len(tla_spec["required_invariants"]),
                "abstraction_count": len(tla_spec["abstractions"]),
                "out_of_model_count": len(tla_spec["out_of_model"]),
            },
        },
        "errors": errors,
    }

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"passed": receipt["passed"], "errors": errors}, sort_keys=True))
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
