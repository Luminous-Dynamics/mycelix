# AC-172 / AC-168: repository-wide Treasury compost compatibility audit

The idempotent outbox change makes receive_compost require a stable effect ID and a matching, committed Payments outbox entry. This alters a cross-zome API contract, so reviewers must not assume that inspected Payments call sites are the only callers in the repository.

## Static guard

Run from the repository root:

~~~sh
python3 -m unittest discover -s scripts/security -p 'test_finance_compost_callsite_audit.py' -v
python3 scripts/security/finance_compost_callsite_audit.py
~~~

The audit:
- scans repository Rust sources for static calls to FunctionName::from("receive_compost");
- requires every discovered call to provide a non-None effect_id in its payload;
- fails if it cannot find any call sites (coverage must be demonstrated);
- checks byte identity for the canonical/workspace Payments coordinator, Payments integrity, Treasury coordinator, Finance types, and finance-wire-types files;
- includes fixture tests for keyed/unkeyed calls, commented-out examples, call-like text inside raw strings, Rust lifetime syntax, nested delimiters in strings, and mirror drift.

This is a static compatibility guard. It does not prove that a call's effect ID is authorized, that every dynamically constructed function name is discovered, that the outbox effect committed, or that Holochain concurrency semantics are correct. Those require source integrity checks and Holochain integration tests.

## Scope boundary

The audit is intentionally fail-closed: a call without effect_id: Some(...) must be migrated or explicitly removed, not silently exempted. Do not add broad allowlists to make the script pass. If a legitimate call path cannot use a Payments outbox, design a separately typed, independently authorized Treasury effect contract instead.
