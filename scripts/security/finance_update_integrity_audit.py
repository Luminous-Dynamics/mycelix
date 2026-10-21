#!/usr/bin/env python3
"""Fail-closed static contract for Finance predecessor-bound update validators."""
from pathlib import Path
import re
import sys

ROOTS = [Path('mycelix-finance/zomes'), Path('mycelix-workspace/mycelix-finance/zomes')]

def mask_rust(source: str) -> str:
    """Blank Rust comments and literals while preserving offsets and newlines.

    This is a small lexical masker, not a full Rust parser. It handles nested block
    comments, escaped regular strings, raw strings, byte strings, character literals,
    and the apostrophe distinction between character literals and lifetimes.
    """
    out = list(source)
    length = len(source)
    index = 0
    state = "code"
    block_depth = 0

    def blank(start: int, end: int) -> None:
        for position in range(start, end):
            if source[position] != "\n":
                out[position] = " "

    while index < length:
        if state == "code":
            if source.startswith("//", index):
                blank(index, index + 2)
                index += 2
                state = "line_comment"
                continue
            if source.startswith("/*", index):
                blank(index, index + 2)
                index += 2
                block_depth = 1
                state = "block_comment"
                continue

            # Raw strings are r###"..."###; raw byte strings are br###"..."###.
            prefix_length = 0
            if source.startswith("br", index):
                prefix_length = 2
            elif source[index] == "r":
                prefix_length = 1
            if prefix_length:
                cursor = index + prefix_length
                while cursor < length and source[cursor] == "#":
                    cursor += 1
                if cursor < length and source[cursor] == '"':
                    hash_count = cursor - (index + prefix_length)
                    terminator = '"' + ("#" * hash_count)
                    closing = source.find(terminator, cursor + 1)
                    literal_end = length if closing < 0 else closing + len(terminator)
                    blank(index, literal_end)
                    index = literal_end
                    continue

            if source[index] == '"':
                blank(index, index + 1)
                index += 1
                state = "string"
                continue

            if source[index] == "'":
                # A lifetime such as 'a or 'static is code, not a character literal.
                # A character literal has a closing apostrophe after one scalar/escape.
                cursor = index + 1
                if cursor < length and (
                    source[cursor].isalpha() or source[cursor] == "_"
                ):
                    while cursor < length and (
                        source[cursor].isalnum() or source[cursor] == "_"
                    ):
                        cursor += 1
                    if cursor >= length or source[cursor] != "'":
                        index += 1
                        continue
                blank(index, index + 1)
                index += 1
                state = "char"
                continue

            index += 1
            continue

        if state == "line_comment":
            if source[index] == "\n":
                state = "code"
            else:
                out[index] = " "
            index += 1
            continue

        if state == "block_comment":
            if source.startswith("/*", index):
                blank(index, index + 2)
                block_depth += 1
                index += 2
            elif source.startswith("*/", index):
                blank(index, index + 2)
                block_depth -= 1
                index += 2
                if block_depth == 0:
                    state = "code"
            else:
                if source[index] != "\n":
                    out[index] = " "
                index += 1
            continue

        # Escaped regular strings and character literals.
        if source[index] == "\\" and index + 1 < length:
            blank(index, index + 2)
            index += 2
        elif (state == "string" and source[index] == '"') or (
            state == "char" and source[index] == "'"
        ):
            blank(index, index + 1)
            index += 1
            state = "code"
        else:
            if source[index] != "\n":
                out[index] = " "
            index += 1

    return "".join(out)


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
