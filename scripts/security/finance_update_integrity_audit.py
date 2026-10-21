#!/usr/bin/env python3
"""Fail-closed static contract for Finance predecessor-bound update validators."""
from pathlib import Path
import re
import sys

ROOTS = [Path('mycelix-finance/zomes'), Path('mycelix-workspace/mycelix-finance/zomes')]

def mask_rust(source: str) -> str:
    out = list(source)
    i = 0
    n = len(source)
    state = 'code'
    while i < n:
        if state == 'code':
            if source.startswith('//', i):
                state = 'line_comment'; out[i] = ' '; out[i+1] = ' '; i += 2; continue
            if source.startswith('/*', i):
                state = 'block_comment'; out[i] = ' '; out[i+1] = ' '; i += 2; continue
            if source[i] == '"':
                state = 'string'; out[i] = ' '; i += 1; continue
            if source[i] == "'":
                state = 'char'; out[i] = ' '; i += 1; continue
            i += 1
        elif state == 'line_comment':
            out[i] = ' '
            if source[i] == '\n': state = 'code'
            i += 1
        elif state == 'block_comment':
            out[i] = ' '
            if source.startswith('*/', i):
                out[i] = ' '; out[i+1] = ' '; i += 2; state = 'code'
            else: i += 1
        elif state == 'string':
            out[i] = ' '
            if source[i] == '\\' and i + 1 < n:
                out[i+1] = ' '; i += 2
            elif source[i] == '"':
                state = 'code'; i += 1
            else: i += 1
        else:
            out[i] = ' '
            if source[i] == '\\' and i + 1 < n:
                out[i+1] = ' '; i += 2
            elif source[i] == "'":
                state = 'code'; i += 1
            else: i += 1
    return ''.join(out)

def braced_block(source: str, start: int) -> str:
    masked = mask_rust(source)
    brace = masked.find("{", start)
    if brace < 0:
        raise ValueError(f"missing opening brace at byte {start}")
    depth = 0
    for i in range(brace, len(masked)):
        if masked[i] == "{":
            depth += 1
        elif masked[i] == "}":
            depth -= 1
            if depth == 0:
                return source[brace:i + 1]
    raise ValueError(f"unterminated block at byte {start}")


def update_arms(source: str):
    masked = mask_rust(source)
    start = masked.find("OpEntry::UpdateEntry")
    if start < 0:
        return
    arrow = masked.find("=>", start)
    if arrow < 0:
        raise ValueError("UpdateEntry arm has no match arrow")
    body_start = masked.find("{", arrow)
    if body_start < 0:
        raise ValueError("UpdateEntry arm has no body")
    block = braced_block(source, body_start)
    masked_block = mask_rust(block)
    matches = list(re.finditer(
        r"EntryTypes::[A-Za-z0-9_]+(?:\([^)]*\))?\s*=>",
        masked_block,
    ))
    for idx, match in enumerate(matches):
        arm = block[match.start():matches[idx + 1].start() if idx + 1 < len(matches) else len(block)]
        yield arm


def functions(source: str):
    masked = mask_rust(source)
    needle = 'fn validate_update_'
    pos = 0
    while True:
        start = masked.find(needle, pos)
        if start < 0: return
        open_brace = masked.find('{', start)
        if open_brace < 0: raise ValueError(f'missing opening brace at byte {start}')
        depth = 0
        end = open_brace
        while end < len(masked):
            if masked[end] == '{': depth += 1
            elif masked[end] == '}':
                depth -= 1
                if depth == 0: break
            end += 1
        if depth != 0: raise ValueError(f'unterminated function at byte {start}')
        line = source.count('\n', 0, start) + 1
        header = source[start:open_brace].strip().splitlines()[0]
        name = header.split('fn ', 1)[1].split('(', 1)[0].strip()
        yield name, line, source[open_brace:end+1]
        pos = end + 1

AUDIT_VERSION = "3"
ROOTS = (
    Path("mycelix-finance/zomes"),
    Path("mycelix-workspace/mycelix-finance/zomes"),
)


def audit_source(path: str, source: str):
    """Audit one Rust source file; return (errors, validator_count)."""
    errors = []
    try:
        update_functions = list(functions(source))
    except ValueError as exc:
        return [f"{path}: parser error: {exc}"], 0

    for name, line, body in update_functions:
        masked_body = mask_rust(body)
        if not re.search(r"\bmust_get_valid_record\s*\(", masked_body):
            errors.append(
                f"{path}:{line}: {name}: missing must_get_valid_record predecessor binding"
            )
            continue

        if not re.search(
            r"\bmust_get_valid_record\s*\(\s*action\.original_action_address\b",
            masked_body,
        ):
            errors.append(
                f"{path}:{line}: {name}: predecessor lookup is not bound to "
                "action.original_action_address"
            )

        if re.search(
            r"if\s+let\s+Ok\s*\([^)]*\)\s*=\s*must_get_valid_record",
            masked_body,
        ):
            errors.append(
                f"{path}:{line}: {name}: swallowed must_get_valid_record predecessor dependency"
            )

        # Inspect each must_get_valid_record call through the end of its containing
        # statement. The question mark must propagate the host dependency's failure.
        for call in re.finditer(r"\bmust_get_valid_record\s*\(", masked_body):
            open_paren = masked_body.find("(", call.start())
            depth = 0
            close_paren = -1
            for pos in range(open_paren, len(masked_body)):
                if masked_body[pos] == "(":
                    depth += 1
                elif masked_body[pos] == ")":
                    depth -= 1
                    if depth == 0:
                        close_paren = pos
                        break
            if close_paren < 0:
                errors.append(
                    f"{path}:{line}: {name}: malformed must_get_valid_record call"
                )
                continue
            semicolon = masked_body.find(";", close_paren + 1)
            statement_end = semicolon if semicolon >= 0 else min(
                len(masked_body), close_paren + 256
            )
            suffix = masked_body[close_paren + 1:statement_end]
            if "?" not in suffix:
                errors.append(
                    f"{path}:{line}: {name}: must_get_valid_record failure is not "
                    "propagated with ?"
                )

    try:
        arms = list(update_arms(source))
    except ValueError as exc:
        errors.append(f"{path}: UpdateEntry parser error: {exc}")
        arms = []

    for arm in arms:
        label = arm.split("=>", 1)[0].strip()
        masked_arm = mask_rust(arm)
        if re.search(r"Ok\(\s*ValidateCallbackResult::Valid", masked_arm):
            errors.append(f"{path}: {label}: direct Valid in UpdateEntry arm")
        validators = re.findall(r"\b(validate_[A-Za-z0-9_]+)\s*\(", masked_arm)
        for validator in validators:
            if not validator.startswith("validate_update_"):
                errors.append(
                    f"{path}: {label}: UpdateEntry arm delegates to non-update validator {validator}"
                )

    return errors, len(update_functions)


def audit_tree(root: Path):
    """Audit both supported Finance trees under a repository root."""
    errors = []
    total_files = 0
    total_validators = 0

    for relative_root in ROOTS:
        absolute_root = root / relative_root
        if not absolute_root.is_dir():
            errors.append(f"{relative_root}: Finance integrity root is missing")
            continue
        files = sorted(absolute_root.glob("*/integrity/src/lib.rs"))
        if not files:
            errors.append(f"{relative_root}: no Finance integrity zomes found")
            continue
        for path in files:
            total_files += 1
            file_errors, validator_count = audit_source(
                path.relative_to(root).as_posix(),
                path.read_text(encoding="utf-8"),
            )
            errors.extend(file_errors)
            total_validators += validator_count

    return errors, total_files, total_validators


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="repository root to audit (defaults to this script's repository)",
    )
    args = parser.parse_args(argv)

    errors, total_files, total_validators = audit_tree(args.root.resolve())
    print(f"FINANCE_UPDATE_PREDECESSOR_AUDIT_VERSION={AUDIT_VERSION}")
    print(
        f"Audited {total_files} Finance integrity zomes and "
        f"{total_validators} validate_update_* functions across {len(ROOTS)} trees."
    )
    if errors:
        print("FINANCE_UPDATE_PREDECESSOR_AUDIT=FAIL")
        for error in errors:
            print("ERROR:", error)
        return 1
    print("FINANCE_UPDATE_PREDECESSOR_AUDIT=PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
