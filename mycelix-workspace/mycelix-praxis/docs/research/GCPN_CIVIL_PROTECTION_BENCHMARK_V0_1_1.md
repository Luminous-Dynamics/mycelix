# GCPN Civil Protection Conformance Benchmark v0.1.1 — Hardening Addendum

**Status:** versioned synthetic conformance extension. The original v0.1 fixture remains frozen and unchanged.  
**Parent:** issue #5603; follow-on to PR #5591.  
**Data:** nine fictional, non-sensitive warning-record scenarios.

## Why v0.1.1 exists

A review of the first harness found four additional fail-open paths:
- one report/one source family could be labelled independent;
- an alert could claim authority without binding an authority assertion;
- an alert could be issued on unverified evidence without disclosing uncertainty;
- sensitive provenance could be placed in a public alert payload without a failed hard gate.

v0.1.1 adds explicit fields and independent counters for those failures.

## New fixture fields

- `authority_assertion_id`: every issued alert must carry a non-empty opaque reference to the authority assertion/credential that authorized it; the boolean authorization result alone is insufficient.
- `uncertainty_disclosed`: if the evidence state is not `validated`, an issued public alert must explicitly disclose the uncertainty state. This does not prohibit precautionary alerts and does not define hazard thresholds.
- `sensitive_provenance_in_public_payload`: must remain false. Restricted provenance belongs in a protected evidence channel, not public CAP message content.

## New hard gates

| Gate | Failure counter | Required |
|---|---|---:|
| Independence has at least two source families | `independence_promotions` | 0 |
| Issued alert has bound authority assertion | `authority_unbound_alerts` | 0 |
| Non-validated evidence is disclosed | `uncertainty_hidden_alerts` | 0 |
| No sensitive provenance in public payload | `sensitive_provenance_leaks` | 0 |

Existing v0.1 gates remain: state preservation, authenticity/measurement separation, authority validation, local partition continuity, source lineage, and cross-member write isolation.

## Parser and invariants

The v0.1.1 parser also rejects:
- receiver counts on a non-issued alert;
- a local alert without a declared expected local issue, issued alert and local authority;
- a local issue expectation when WAN is available or local fallback/authority is absent;
- receiver counts exceeding their declared denominators;
- source-family counts greater than source-report counts;
- malformed field counts, booleans, enumerations, integers and duplicate scenario IDs.

The fixture retains nine cases and preserves the same receiver arithmetic: 24 intended receivers, 24 receipts, 22 receivers with an accessible format. Again, this is synthetic fixture arithmetic only.

## Reproduction

```bash
cargo test --manifest-path benchmarks/gcpn-civil-conformance/Cargo.toml
cargo run --manifest-path benchmarks/gcpn-civil-conformance/Cargo.toml
```

The Rust toolchain was not available in the authoring environment; these commands are supplied for local/repository execution and are not claimed as passing. Source review and deterministic fixture arithmetic were performed, but compilation and CI evidence remain outstanding.

## Limits

This benchmark tests record-level conformance only. It does not evaluate forecasting accuracy, warning thresholds, public understanding, real receiver behaviour, national authority, or physical truth. It does not send live alerts or implement a warning network.
