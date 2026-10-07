#!/usr/bin/env python3
from pathlib import Path
import re
import sys

ROOTS = [Path('mycelix-finance/zomes'), Path('mycelix-workspace/mycelix-finance/zomes')]
ALLOWLIST = {
    'lending::validate_update_loan': 'deprecated lending surface',
    'lending::validate_update_loan_offer': 'deprecated lending surface',
    'lending::validate_update_payment_schedule': 'deprecated lending surface',
}
FUNCTION_RE = re.compile(r'(?m)^fn (validate_update_[A-Za-z0-9_]+)\s*\(')

def body(source, start):
    brace = source.find('{', start)
    if brace < 0: raise ValueError('missing opening brace')
    depth = 0
    for i in range(brace, len(source)):
        if source[i] == '{': depth += 1
        elif source[i] == '}':
            depth -= 1
            if depth == 0: return source[brace:i+1]
    raise ValueError('unterminated function')

errors = []
files = sorted(p for r in ROOTS if r.exists() for p in r.glob('*/integrity/src/lib.rs'))
for path in files:
    src = path.read_text(encoding='utf-8')
    zome = path.parts[-3]
    for m in FUNCTION_RE.finditer(src):
        name = m.group(1)
        b = body(src, m.start())
        key = f'{zome}::{name}'
        if re.search(r'if\s+let\s+Ok\s*\([^)]*\)\s*=\s*must_get_valid_record', b):
            errors.append(f'{path}: {name}: swallowed must_get_valid_record dependency')
        elif 'must_get_valid_record' not in b and key not in ALLOWLIST:
            errors.append(f'{path}: {name}: no exact predecessor dependency')
print(f'Audited {len(files)} Finance integrity zomes across {len(ROOTS)} trees.')
if errors:
    print('\n'.join('ERROR: '+e for e in errors))
    sys.exit(1)
print('FINANCE_UPDATE_PREDECESSOR_AUDIT=PASS')