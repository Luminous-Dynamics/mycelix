# FIN-ACC-REVIEW-001 — exact-subject review verifier v1.0.0

This is the first executable, deliberately narrow tranche of [#5852](https://github.com/Luminous-Dynamics/mycelix/issues/5852).

## Paradigm

Review evidence is bound to an immutable workpaper subject and review-policy digest. A mutable `approved` boolean or a populated `approved_by` array is not a review theorem.

The subject digest covers the exact workpaper reference/content digest, reporting entity/period/book domain, assertion-risk pairs, performed procedure, source/evidence frontiers, parent workpaper digests, accounting profile/policy, review-policy reference/digest, knowledge/authority frontiers and preparer identity/type.

Each review event binds both the computed subject digest and the computed review-policy digest, plus role, canonical principal reference, decision, entity/period/assertion/procedure scope, authority frontier, and opaque identity/authority evidence references.

## Canonical digests

The verifier uses domain-separated SHA-256 over compact canonical JSON:
- object keys sorted by UTF-8 byte order;
- arrays preserve their authored order;
- JSON strings use standard JSON escaping and UTF-8 output;
- numbers retain their JSON lexical value as parsed by `serde_json`;
- no insignificant whitespace.

Domains:
- `mycelix:fin-acc-review:subject:v1\0`
- `mycelix:fin-acc-review:policy:v1\0`

This is a local review-subject commitment convention, not a signature format or general Mycelix canonicalization primitive. Future composition with a shared canonical semantic waist should replace this local encoding rather than silently assume equivalence.

## Structural results only

The strongest output is `structurally_ready_pending_external_authority`. The report always sets:
- `external_authority_verified = false`
- `human_review_sufficiency_verified = false`
- `can_approve_close = false`

The verifier compares opaque principal references for exact equality. It does not resolve aliases, authenticate principals, verify signatures, determine current role membership, validate delegation, prove reviewers read/understood the material, or authorize journal posting / close release / tax filing. Identity and authority references are only required to exist structurally. Those checks remain in the Business Fabric identity/authority/revalidation stack (#566/#570/#571) and qualified adapters.

v1.0.0 supports preparer/performer/reviewer/approver/policy-owner role references, policy-declared role disjointness, minimum unique-principal counts, and an explicit `non_human_eligible_roles` allowlist. Review events identify `human`, `agent` or `system` actor kind; non-human events cannot satisfy a role unless that exact role is explicitly allowlisted by the exact review policy. Exact duplicate event replay is idempotent; reusing an event ID with changed event content is a conflict. Delegation, revocation, withdrawal/supersession lifecycle and dynamic authority checks are intentionally unsupported; do not infer them.

## Run

From this directory:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --input ../fixtures/qualified-structural-review.example.json --out /tmp/review-report.json
```

Candidate records are validated against `FIN_ACCOUNTING_REVIEW_V1.0.0.schema.json`. The report is a machine-checkable structural disposition, not an assurance opinion.

## Claim ceiling

Passing synthetic tests proves only the exact input/schema/verifier behaviors. It does not prove real-world role authority, professional review sufficiency, completeness of the workpaper, audit/ISA/IFRS/tax compliance, or authority to post/close/file. The result must never be interpreted as a permission to approve a close.


The v1.0.0 synthetic negative control `agent-review-blocked.example.json` proves that an agent cannot satisfy the reviewer role by default. A separate unit test demonstrates that a policy can explicitly allow a non-human principal for a narrowly scoped role; that remains only structural policy interpretation, not authority verification.


Non-human safety invariant: policy may allow an agent/system to produce or perform work, but cannot promote it into the human reviewer or approver role. Agent and system events must bind their exact actor build digest. External authority remains unverified for every disposition.
