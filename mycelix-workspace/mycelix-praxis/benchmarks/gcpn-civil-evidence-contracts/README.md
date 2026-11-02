# GCPN civil-evidence contracts v0.1

This crate extends the six-stage compositional auditor with separate, typed civil-event claims. It consumes an existing compositional `AuditReport` and requires named base claims to be supporting before downstream delivery/action claims can support.

## The evidence graph

The contract model deliberately separates:

1. **Authority assertion** — recognized registry status, verified signature, jurisdiction, hazard scope, validity interval, CAP originator/message/version and decision binding. A cryptographic signature alone is not treated as legal authority.
2. **Channel delivery** — provider acceptance, device delivery, failure, delay, unknown and duplicate receipts. Delivery is reported per intended cohort/channel target. Cross-channel counts are not collapsed into unique people; that requires a separate privacy-preserving reach contract.
3. **Accessibility** — a tested language/format/assistive-technology profile tied to the exact device-delivery receipt for the message.
4. **Comprehension and action intent** — separate measured rates tied to the same device receipt and an accessible-presentation record. Accessibility does not imply comprehension.
5. **Action execution** — approved, dispatched, started, completed, failed, cancelled and unknown receipts. Completion requires the exact decision, authority assertion, executor and action type. Approval is not completion.
6. **Case-frame accounting** — unique case keys with observed, missing, disputed and excluded dispositions. Missing outcomes remain in the eligible denominator; duplicate keys fail closed.

The audit also consumes explicit supporting claims from the compositional assurance engine:
- CAP lifecycle support gates delivery claims.
- Decision and authority support gate authorized-action claims.
- The base audit cutoff must match the civil-evidence cutoff.

## Typed semantics

The output is intentionally non-compensatory. Each evidence family returns its own Supports, Refutes, Indeterminate or Unavailable disposition. There is no aggregate score.

Post-cutoff records are excluded from as-of counts and add a violation. Unknown or missing evidence does not become a negative outcome or a successful event. Device delivery is not proof of comprehension; comprehension or intention is not proof of completed action.

Only aggregate cohort counts are represented; no personal receiver identifiers are part of the schema. The declared authority-registry state is metadata from the configured source, not an independent legal determination.

## Run

    cargo fmt --all -- --check
    cargo test --locked --offline
    cargo clippy --locked --offline --all-targets -- -D warnings
    cargo run --locked --offline

Rust 1.98.1 is pinned. The only dependency is the local `gcpn-compositional-assurance` crate.

## Qualification boundary

All fixtures are synthetic. This is not a warning issuer, transport, recipient tracker, dispatch service, legal authority, national MHEWS certification or evidence of disaster impact. Real use requires national authority agreement, validated registry/transport interfaces, privacy and purpose limitation, local accessibility testing, and field protocols that distinguish measured comprehension from actual action.

The companion adapter and contract are designed to replace five previously unrepresented fault dimensions: authority mismatch, delivery receipt incompleteness, accessibility unverified, action receipt missing and duplicate case-frame keys.