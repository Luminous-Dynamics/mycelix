#!/usr/bin/env python3
"""Fail-closed audit for Treasury compost API call sites and mirrored Finance sources.

Run from the repository root:
    python3 scripts/security/finance_compost_callsite_audit.py

This is a static compatibility guard. It does not prove runtime authorization,
exactly-once behavior, or Holochain source-chain concurrency.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

MIRRORED_PAIRS = (
    (
        "mycelix-finance/zomes/payments/coordinator/src/lib.rs",
        "mycelix-workspace/mycelix-finance/zomes/payments/coordinator/src/lib.rs",
    ),
    (
        "mycelix-finance/zomes/payments/integrity/src/lib.rs",
        "mycelix-workspace/mycelix-finance/zomes/payments/integrity/src/lib.rs",
    ),
    (
        "mycelix-finance/zomes/treasury/coordinator/src/lib.rs",
        "mycelix-workspace/mycelix-finance/zomes/treasury/coordinator/src/lib.rs",
    ),
    (
        "mycelix-finance/types/src/lib.rs",
        "mycelix-workspace/mycelix-finance/types/src/lib.rs",
    ),
    (
        "mycelix-finance/crates/finance-wire-types/src/lib.rs",
        "mycelix-workspace/mycelix-finance/crates/finance-wire-types/src/lib.rs",
    ),
)

CALL_MARKER = re.compile(
    r'\bFunctionName\s*::\s*from\s*\(\s*"receive_compost"\s*\)'
)
CALL_START = re.compile(r"\bcall\s*\(")
EFFECT_ID = re.compile(r"\beffect_id\s*:\s*Some\s*\(")
POST_COMMIT_MARKER = re.compile(
    r"#\s*\[\s*hdk_extern\s*\(\s*infallible\s*\)\s*\]\s*"
    r"pub\s+fn\s+post_commit\s*\([^)]*\)\s*\{",
    re.DOTALL,
)
POST_COMMIT_NAME = re.compile(r"\bfn\s+post_commit\s*\(")


def mask_rust_source(source: str, *, keep_string_literals: bool) -> str:
    """Mask comments and (optionally) string/char literals, preserving offsets."""
    chars = list(source)
    length = len(source)
    i = 0

    def blank_span(start: int, end: int) -> None:
        for pos in range(start, min(end, length)):
            if chars[pos] not in "\r\n":
                chars[pos] = " "

    while i < length:
        if source.startswith("//", i):
            end = source.find("\n", i)
            if end < 0:
                end = length
            blank_span(i, end)
            i = end
            continue

        if source.startswith("/*", i):
            start = i
            depth = 1
            i += 2
            while i < length and depth:
                if source.startswith("/*", i):
                    depth += 1
                    i += 2
                elif source.startswith("*/", i):
                    depth -= 1
                    i += 2
                else:
                    i += 1
            blank_span(start, i)
            continue

        # Raw string forms: r"..." / r#"..."# and byte raw strings br#"..."#.
        raw_match = re.match(r"(?:br|r)(#{0,32})\"", source[i:])
        if raw_match:
            hashes = raw_match.group(1)
            start = i
            opener_len = len(raw_match.group(0))
            close_token = '"' + hashes
            close_at = source.find(close_token, i + opener_len)
            end = length if close_at < 0 else close_at + len(close_token)
            if not keep_string_literals:
                blank_span(start, end)
            i = end
            continue

        # Byte strings and ordinary quoted strings.
        quote_pos = None
        if source.startswith('b"', i):
            quote_pos = i + 1
        elif source[i] == '"':
            quote_pos = i
        if quote_pos is not None:
            start = i
            i = quote_pos + 1
            escaped = False
            while i < length:
                char = source[i]
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == '"':
                    i += 1
                    break
                i += 1
            if not keep_string_literals:
                blank_span(start, i)
            continue

        # Character literals are short and closed; don't confuse Rust lifetimes
        # such as &'a or 'static with a quoted character.
        if source[i] == "'":
            char_end = i
            if i + 2 < length and source[i + 2] == "'":
                char_end = i + 3
            elif i + 1 < length and source[i + 1] == "\\":
                closing = source.find("'", i + 2, min(length, i + 20))
                if closing >= 0 and "\n" not in source[i:closing] and "\r" not in source[i:closing]:
                    char_end = closing + 1
            if char_end > i:
                if not keep_string_literals:
                    blank_span(i, char_end)
                i = char_end
                continue

        i += 1

    return "".join(chars)


def matching_paren(code_mask: str, open_paren: int) -> int | None:
    """Return the closing parenthesis paired with open_paren."""
    if open_paren >= len(code_mask) or code_mask[open_paren] != "(":
        return None
    depth = 0
    for index in range(open_paren, len(code_mask)):
        if code_mask[index] == "(":
            depth += 1
        elif code_mask[index] == ")":
            depth -= 1
            if depth == 0:
                return index
    return None


def compost_callsite_issues(path: str, source: str) -> tuple[list[str], int]:
    """Require every statically named receive_compost call to carry effect_id: Some(...)."""
    comment_mask = mask_rust_source(source, keep_string_literals=True)
    code_mask = mask_rust_source(source, keep_string_literals=False)
    issues: list[str] = []
    # The marker includes a quoted function name. Filter out lookalikes embedded
    # inside a larger Rust string/raw-string literal using the fully masked view.
    matches = [
        match
        for match in CALL_MARKER.finditer(comment_mask)
        if code_mask[match.start():match.start() + len("FunctionName")] == "FunctionName"
    ]

    for number, match in enumerate(matches, start=1):
        starts = [item for item in CALL_START.finditer(code_mask, 0, match.start())]
        if not starts:
            issues.append(f"{path}: receive_compost call site {number} has no enclosing call(...)")
            continue
        call_start = starts[-1]
        open_paren = code_mask.find("(", call_start.start(), call_start.end())
        close_paren = matching_paren(code_mask, open_paren)
        if close_paren is None:
            issues.append(f"{path}: receive_compost call site {number} has unbalanced call(...)")
            continue
        arguments = comment_mask[match.end():close_paren]
        if not EFFECT_ID.search(arguments):
            issues.append(
                f"{path}: receive_compost call site {number} lacks effect_id: Some(...); "
                "legacy/unkeyed Treasury credits are not admissible"
            )

    return issues, len(matches)


def post_commit_contract_issues(path: str, source: str) -> list[str]:
    """Enforce Holochain's infallible, no-return post_commit callback contract."""
    code_mask = mask_rust_source(source, keep_string_literals=False)
    declarations = list(POST_COMMIT_NAME.finditer(code_mask))
    if len(declarations) != 1:
        return [
            f"{path}: expected exactly one post_commit declaration; found {len(declarations)}"
        ]
    valid = list(POST_COMMIT_MARKER.finditer(code_mask))
    if len(valid) != 1:
        return [
            f"{path}: post_commit must use #[hdk_extern(infallible)] and have no return type"
        ]
    return []


def audit_repository(root: Path) -> tuple[list[str], int, list[str]]:
    issues: list[str] = []
    mirror_passes: list[str] = []

    for left_rel, right_rel in MIRRORED_PAIRS:
        left = root / left_rel
        right = root / right_rel
        if not left.is_file():
            issues.append(f"missing canonical Finance mirror: {left_rel}")
            continue
        if not right.is_file():
            issues.append(f"missing workspace Finance mirror: {right_rel}")
            continue
        left_source = left.read_text(encoding="utf-8")
        right_source = right.read_text(encoding="utf-8")
        if left.read_bytes() != right.read_bytes():
            issues.append(f"Finance mirrors are not byte-identical: {left_rel} != {right_rel}")
        else:
            mirror_passes.append(left_rel)

        if left_rel.endswith("payments/coordinator/src/lib.rs"):
            issues.extend(post_commit_contract_issues(left_rel, left_source))
            issues.extend(post_commit_contract_issues(right_rel, right_source))

    total_calls = 0
    excluded_parts = {".git", "target", "node_modules", "vendor"}
    for path in sorted(root.rglob("*.rs")):
        try:
            relative = path.relative_to(root)
        except ValueError:
            continue
        if excluded_parts.intersection(relative.parts):
            continue
        try:
            source = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError) as exc:
            issues.append(f"cannot read {relative}: {exc}")
            continue
        found_issues, count = compost_callsite_issues(relative.as_posix(), source)
        issues.extend(found_issues)
        total_calls += count

    if total_calls == 0:
        issues.append('no static FunctionName::from("receive_compost") call sites found; audit cannot establish coverage')

    return issues, total_calls, mirror_passes


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    issues, total_calls, mirror_passes = audit_repository(root)
    if issues:
        print(f"FAIL: {len(issues)} compost-callsite/mirror issue(s)")
        for issue in issues:
            print(f" - {issue}")
        return 1
    print(f"PASS: {total_calls} receive_compost call site(s) provide effect_id: Some(...)")
    print(f"PASS: {len(mirror_passes)} canonical/workspace Finance file pairs are byte-identical")
    print("BOUNDARY: static compatibility audit only; not runtime or Holochain concurrency qualification")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
