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
    block = braced_block(source, start)
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

errors = []
files = sorted(p for root in ROOTS if root.exists() for p in root.glob('*/integrity/src/lib.rs'))
for path in files:
    source = path.read_text(encoding='utf-8')
    zome = path.parts[-4]
    try:
        funcs = list(functions(source))
    except ValueError as exc:
        errors.append(f'{path}: parser error: {exc}')
        continue
    for name, line, body in funcs:
        if 'must_get_valid_record' not in body:
            errors.append(f'{path}:{line}: {name}: missing must_get_valid_record predecessor binding')
        elif re.search(r'if\s+let\s+Ok\s*\([^)]*\)\s*=\s*must_get_valid_record', body):
            errors.append(f'{path}:{line}: {name}: swallowed must_get_valid_record predecessor dependency')
    try:
        arms = list(update_arms(source))
    except ValueError as exc:
        errors.append(f'{path}: UpdateEntry parser error: {exc}')
        arms = []
    for arm in arms:
        label = arm.split('=>', 1)[0].strip()
        if re.search(r'Ok\(\s*ValidateCallbackResult::Valid', arm):
            errors.append(f'{path}: {label}: direct Valid in UpdateEntry arm')
        validators = re.findall(r'\b(validate_[A-Za-z0-9_]+)\s*\(', arm)
        for validator in validators:
            if validator != 'validate_update_' and not validator.startswith('validate_update_'):
                errors.append(f'{path}: {label}: UpdateEntry arm delegates to non-update validator {validator}')

print(f'Audited {len(files)} Finance integrity zomes across {len(ROOTS)} trees.')
if errors:
    print('FINANCE_UPDATE_PREDECESSOR_AUDIT=FAIL')
    for error in errors: print('ERROR:', error)
    sys.exit(1)
print('FINANCE_UPDATE_PREDECESSOR_AUDIT=PASS')