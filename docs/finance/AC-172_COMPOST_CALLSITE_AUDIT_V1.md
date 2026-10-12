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

The same guard checks the Payments zome's `post_commit` callback uses Holochain's `#[hdk_extern(infallible)]` ABI with no return value. The implementation was corrected to this contract because Holochain invokes `post_commit` after successful source-chain commit, disallows writes from the hook, and requires it to handle follow-up errors itself. See the official [callback/lifecycle hook contract](https://developer.holochain.org/build/callbacks-and-lifecycle-hooks/) and [atomic zome function transaction model](https://developer.holochain.org/build/zome-functions/). The callback additionally triggers only when the committed actions include a fresh PendingCompost queue tag (non-empty effect ID and retry_count = 0). A failed delivery replaces the tag with an increased retry count, and that metadata update does not itself retrigger the drain. A Rust regression test covers fresh versus retried tags.

This is a static compatibility guard. It does not prove that a call's effect ID is authorized, that every dynamically constructed function name is discovered, that the outbox effect committed, or that Holochain concurrency semantics are correct. Those require source integrity checks and Holochain integration tests.

## Scope boundary

The audit is intentionally fail-closed: a call without effect_id: Some(...) must be migrated or explicitly removed, not silently exempted. Do not add broad allowlists to make the script pass. If a legitimate call path cannot use a Payments outbox, design a separately typed, independently authorized Treasury effect contract instead.
