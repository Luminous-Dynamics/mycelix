#!/usr/bin/env python3
# Copyright (C) 2024-2026 Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Fail-closed caller census for removing payments::credit_sap as a public zome ABI.

Cross-zome calls are name-dispatched and can survive Rust compilation after an
extern is made private. Search all Finance coordinator sources except the
Payments coordinator itself, verify the ABI is private in both projections,
and require canonical/workspace caller inventories to agree. Any remaining
external reference blocks qualification; do not whitelist known callers merely
to turn the check green.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# Match the exact function-name literal, independent of which Holochain
# constructor/conversion API wraps it. This intentionally errs toward a false
# positive: any external coordinator literal should be reviewed before the ABI is
# removed. It also catches Rust raw-string spellings such as r#"credit_sap"#.
RAW_CREDIT_REFERENCE = re.compile(r'(?:r#*)?"credit_sap"#*')
# Match Rust raw and ordinary string tokens when reviewing compile-time concat! calls.
# concat!("credit_", "sap") is a static function name, not runtime assembly.
RUST_STRING_LITERAL = re.compile(
    r'r(?P<hashes>#{0,})"(?P<raw>.*?)"(?P=hashes)|(?P<quoted>"(?:\\.|[^"\\])*")',
    re.DOTALL,
)
CONCAT_MACRO = re.compile(r"\bconcat!\s*\(")
# Permit other outer attributes between hdk_extern and the function declaration.
# Stop at the first function declaration so an unrelated extern cannot make a later
# private helper look externally exported.
PUBLIC_RAW_CREDIT_ABI = re.compile(
    r"#\s*\[\s*hdk_extern\s*\]"
    r"(?:(?:\s*#\s*\[[^\]]*\])|(?:\s*//[^\n]*(?:\n|$))|(?:\s*/\*.*?\*/))*"
    r"\s*(?:pub\s+)?fn\s+credit_sap\s*\(",
    re.DOTALL,
)


def has_public_raw_credit_abi(source_text: str) -> bool:
    """Return whether raw credit is still declared as a Holochain extern."""
    return PUBLIC_RAW_CREDIT_ABI.search(source_text) is not None


def _skip_rust_trivia(text: str, index: int) -> int:
    """Skip separators and Rust comments between concat! string literal arguments."""
    while index < len(text):
        if text[index].isspace() or text[index] == ",":
            index += 1
            continue
        if text.startswith("//", index):
            newline = text.find("\n", index + 2)
            if newline < 0:
                return len(text)
            index = newline + 1
            continue
        if text.startswith("/*", index):
            # Rust block comments may nest; avoid interpreting comment text as a token.
            depth = 1
            cursor = index + 2
            while cursor < len(text) and depth:
                if text.startswith("/*", cursor):
                    depth += 1
                    cursor += 2
                elif text.startswith("*/", cursor):
                    depth -= 1
                    cursor += 2
                else:
                    cursor += 1
            if depth:
                return len(text) + 1
            index = cursor
            continue
        break
    return index


def _literal_value(match: re.Match[str]) -> str | None:
    """Decode only the Rust string escapes understood by this static name scanner."""
    if match.group("raw") is not None:
        return match.group("raw")

    spelling = match.group("quoted")
    if spelling is None or len(spelling) < 2:
        return None
    body = spelling[1:-1]
    decoded: list[str] = []
    index = 0
    simple_escapes = {
        "\\": "\\",
        '"': '"',
        "'": "'",
        "n": "\n",
        "r": "\r",
        "t": "\t",
        "0": "\0",
    }
    while index < len(body):
        char = body[index]
        if char != "\\":
            decoded.append(char)
            index += 1
            continue
        index += 1
        if index >= len(body):
            return None
        escape = body[index]
        if escape in simple_escapes:
            decoded.append(simple_escapes[escape])
            index += 1
            continue
        if escape == "x":
            digits = body[index + 1 : index + 3]
            if len(digits) != 2 or not re.fullmatch(r"[0-9a-fA-F]{2}", digits):
                return None
            codepoint = int(digits, 16)
            # Rust text-string \\x escapes are ASCII-only.
            if codepoint > 0x7F:
                return None
            decoded.append(chr(codepoint))
            index += 3
            continue
        if escape == "u" and body[index + 1 : index + 2] == "{":
            close = body.find("}", index + 2)
            if close < 0:
                return None
            digits = body[index + 2 : close]
            if not digits or not re.fullmatch(r"[0-9a-fA-F_]+", digits):
                return None
            try:
                codepoint = int(digits.replace("_", ""), 16)
                if codepoint > 0x10FFFF or 0xD800 <= codepoint <= 0xDFFF:
                    return None
                decoded.append(chr(codepoint))
            except (ValueError, OverflowError):
                return None
            index = close + 1
            continue
        # Rust escaped-newline continuation: discard the line break and following
        # ASCII whitespace. This is static normalization, not arbitrary evaluation.
        if escape == "\n":
            index += 1
            while index < len(body) and body[index] in " \t\r\n":
                index += 1
            continue
        if escape == "\r" and body[index + 1 : index + 2] == "\n":
            index += 2
            while index < len(body) and body[index] in " \t\r\n":
                index += 1
            continue
        return None
    return "".join(decoded)


def _static_decoded_literal_hits(source_text: str) -> list[tuple[int, int, str]]:
    """Find escaped normal string literals that decode to the raw-credit name."""
    hits: list[tuple[int, int, str]] = []
    for literal in RUST_STRING_LITERAL.finditer(source_text):
        spelling = literal.group(0)
        # Exact normal/raw literals already have their own finding.
        if RAW_CREDIT_REFERENCE.fullmatch(spelling):
            continue
        if _literal_value(literal) == "credit_sap":
            line_number = source_text.count("\n", 0, literal.start()) + 1
            hits.append(
                (
                    literal.start(),
                    line_number,
                    f'{spelling} => decoded static string "credit_sap"',
                )
            )
    return hits


def _find_macro_close(source_text: str, opening: int) -> int:
    """Find a matching ')' without counting parentheses inside literals/comments."""
    depth = 1
    cursor = opening + 1
    while cursor < len(source_text):
        if source_text.startswith("//", cursor):
            newline = source_text.find("\n", cursor + 2)
            cursor = len(source_text) if newline < 0 else newline + 1
            continue
        if source_text.startswith("/*", cursor):
            depth_comment = 1
            cursor += 2
            while cursor < len(source_text) and depth_comment:
                if source_text.startswith("/*", cursor):
                    depth_comment += 1
                    cursor += 2
                elif source_text.startswith("*/", cursor):
                    depth_comment -= 1
                    cursor += 2
                else:
                    cursor += 1
            if depth_comment:
                return -1
            continue
        literal = RUST_STRING_LITERAL.match(source_text, cursor)
        if literal is not None:
            cursor = literal.end()
            continue
        if source_text[cursor] == "(":
            depth += 1
        elif source_text[cursor] == ")":
            depth -= 1
            if depth == 0:
                return cursor
        cursor += 1
    return -1


def _static_concat_hits(source_text: str) -> list[tuple[int, int, str]]:
    """Find concat! calls whose string-literal arguments resolve to credit_sap."""
    hits: list[tuple[int, int, str]] = []
    direct_spans = [match.span() for match in RAW_CREDIT_REFERENCE.finditer(source_text)]
    for macro in CONCAT_MACRO.finditer(source_text):
        opening = source_text.find("(", macro.start(), macro.end())
        if opening < 0:
            continue
        # Find the matching close without confusing ')' inside string literals
        # or comments for the macro delimiter.
        closing = _find_macro_close(source_text, opening)
        if closing < 0:
            continue
        end = closing + 1
        # An exact literal already has its own finding; avoid duplicate hits.
        if any(start >= macro.start() and stop <= end for start, stop in direct_spans):
            continue
        body = source_text[opening + 1 : closing]
        values: list[str] = []
        cursor = _skip_rust_trivia(body, 0)
        valid = True
        while cursor < len(body):
            literal = RUST_STRING_LITERAL.match(body, cursor)
            if literal is None:
                valid = False
                break
            value = _literal_value(literal)
            if value is None:
                valid = False
                break
            values.append(value)
            cursor = _skip_rust_trivia(body, literal.end())
        if valid and values and "".join(values) == "credit_sap":
            line_number = source_text.count("\n", 0, macro.start()) + 1
            source_spelling = source_text[macro.start() : end]
            hits.append(
                (
                    macro.start(),
                    line_number,
                    f'{source_spelling} => static function name "credit_sap"',
                )
            )
    return hits


def scan(zomes_root: Path) -> list[tuple[str, int, str]]:
    """Return exact raw-credit name literals outside payments coordinator."""
    hits: list[tuple[str, int, str]] = []
    for source in sorted(zomes_root.rglob("*.rs")):
        relative = source.relative_to(zomes_root)
        parts = relative.parts

        # Private intra-coordinator helper calls are not cross-zome ABI consumers.
        if len(parts) >= 2 and parts[0:2] == ("payments", "coordinator"):
            continue
        if "coordinator" not in parts:
            continue

        try:
            source_text = source.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            raise RuntimeError(f"cannot read {source}: {exc}") from exc

        # Search the complete source file for exact normal/raw string literals,
        # independent of constructor spelling or line breaks. This errs toward
        # false positives (including comments) so each occurrence receives review.
        file_hits = [
            (
                match.start(),
                source_text.count("\n", 0, match.start()) + 1,
                match.group(0),
            )
            for match in RAW_CREDIT_REFERENCE.finditer(source_text)
        ]
        # Also inspect escaped literals and statically composed Rust concat! calls.
        # Arbitrary const indirection or runtime string assembly remains outside.
        file_hits.extend(_static_decoded_literal_hits(source_text))
        file_hits.extend(_static_concat_hits(source_text))
        for _offset, line_number, spelling in sorted(file_hits):
            hits.append((relative.as_posix(), line_number, spelling))
    return hits


def compare_projection_files(canonical_root: Path, workspace_root: Path) -> list[tuple[str, str]]:
    """Return byte-level file differences between the canonical and workspace zome trees.

    These trees are maintained as mirrored projections. A caller-only census can
    miss security-relevant drift elsewhere (for example, an ABI or validator change
    in one projection), so qualification requires their relative file inventories
    and bytes to remain identical. Symlinks are rejected rather than followed.
    """
    def inventory(root: Path) -> dict[str, bytes]:
        files: dict[str, bytes] = {}
        for path in sorted(root.rglob("*")):
            if path.is_symlink():
                raise RuntimeError(f"unexpected symlink in Finance zome projection: {path}")
            if not path.is_file():
                continue
            relative = path.relative_to(root).as_posix()
            try:
                files[relative] = path.read_bytes()
            except OSError as exc:
                raise RuntimeError(f"cannot read projection file {path}: {exc}") from exc
        return files

    for root in (canonical_root, workspace_root):
        if root.is_symlink():
            raise RuntimeError(f"unexpected symlink used as Finance zome projection root: {root}")

    canonical = inventory(canonical_root)
    workspace = inventory(workspace_root)
    differences: list[tuple[str, str]] = []
    for relative in sorted(set(canonical) | set(workspace)):
        if relative not in canonical:
            differences.append((relative, "missing from canonical projection"))
        elif relative not in workspace:
            differences.append((relative, "missing from workspace projection"))
        elif canonical[relative] != workspace[relative]:
            differences.append((relative, "content differs"))
    return differences


def audit(root: Path = ROOT) -> int:
    """Run the whole fail-closed ABI and caller audit under a project root."""
    canonical_zomes = root / "mycelix-finance" / "zomes"
    workspace_zomes = root / "mycelix-workspace" / "mycelix-finance" / "zomes"
    missing = [path for path in (canonical_zomes, workspace_zomes) if not path.is_dir()]
    if missing:
        for path in missing:
            print(f"ERROR: required Finance source root missing: {path}", file=sys.stderr)
        return 2

    try:
        # Run before reading source files so no source symlink is followed by the
        # ABI or caller scan. Content differences are reported after specific ABI
        # and caller diagnostics; symlink/read failures stop the audit immediately.
        projection_differences = compare_projection_files(canonical_zomes, workspace_zomes)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    # Audit the ABI itself as well as its consumers. Otherwise the caller census
    # could pass while an unrestricted raw-credit extern is accidentally restored.
    payments_coordinators = (
        root / "mycelix-finance" / "zomes" / "payments" / "coordinator" / "src" / "lib.rs",
        root / "mycelix-workspace" / "mycelix-finance" / "zomes" / "payments" / "coordinator" / "src" / "lib.rs",
    )
    for path in payments_coordinators:
        try:
            source_text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            print(f"ERROR: cannot read required Payments coordinator {path}: {exc}", file=sys.stderr)
            return 2
        if has_public_raw_credit_abi(source_text):
            print(f"FAIL: raw payments::credit_sap remains exposed as a Holochain extern: {path}")
            print("Remove the extern only with its source-specific caller migration and runtime coverage.")
            return 1

    try:
        canonical = scan(canonical_zomes)
        workspace = scan(workspace_zomes)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if canonical != workspace:
        print("FAIL: canonical and workspace raw-credit caller inventories differ.")
        print(f"canonical: {canonical}")
        print(f"workspace: {workspace}")
        return 1

    if projection_differences:
        print("FAIL: canonical and workspace Finance zome file projections differ.")
        for relative, reason in projection_differences:
            print(f"  {relative}: {reason}")
        print(f"Found {len(projection_differences)} projection difference(s).")

    if canonical:
        print("FAIL: external coordinator sources still contain a raw-credit function-name reference.")
        print("Review every occurrence; do not qualify removal of the public ABI until each")
        print("is migrated to source-specific authorization or deliberately disabled.")
        for relative, line_number, expression in canonical:
            print(f"  {relative}:{line_number}: {expression}")
        print(f"Found {len(canonical)} matching function-name reference(s) in each Finance projection.")

    if canonical or projection_differences:
        return 1

    print("PASS: raw credit ABI is private, the two Finance zome projections are byte-identical, and no direct, escaped/Unicode-escaped, or statically concatenated raw-credit function-name reference remains outside Payments coordinator.")
    print("This source scanner does not resolve arbitrary const aliases, every possible macro/literal transformation, or runtime-constructed names; it also does not prove SAP conservation or exactly-once settlement.")
    return 0


def main() -> int:
    return audit(ROOT)


if __name__ == "__main__":
    raise SystemExit(main())
