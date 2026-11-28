# Craft Decision Basis Capsule (DBC) structural oracle V0.1

Test-only, independent Go reference for composing exact DJCC owner receipts into a Craft water-service decision basis. This does not implement DJCC, verify signatures, authenticate evidence, decide legal authority, evaluate water policy, or adjudicate physical outcomes.

## Design boundary

The DBC is a content-addressed envelope of references to independently owned receipts. It is not another event log, temporal database, trust registry, provenance store, policy evaluator, authorization kernel, execution service or outcome adjudicator.

The validator checks only the declared V0.1 structural contract:

- all eight DJCC stages appear exactly once;
- fixed producer/consumer dependency edges match the DJCC full evidence-to-outcome profile;
- every consumer input binds the exact producer receipt ID, output digest and schema ID;
- stage/receipt IDs and input slots are unique;
- causal event frontier and progress antichain have distinct type/schema references;
- completeness is not promoted from a valid event frontier without an independently established owner-universe claim and stable progress antichain;
- positive independence requires both a failure-domain graph and a separately supplied lineage-attestation reference;
- positive authority requires a named principal, scoped capability reference and established authority receipt;
- action coverage binds the exact immutable plan digest and revision;
- positive effect requires qualified outcome evidence and resolved reconciliation;
- final claims cannot exceed their profile-owned producer stages or relevant external receipts.

A passing structural check never authenticates any referenced receipt. Trust owners remain responsible for cryptographic and institutional verification.

## Run

```sh
gofmt -w oracle/*.go
go test ./...
go test -race ./...
go vet ./...
go test -cover ./...
```

## Fixture suite

`fixtures/dbc-known-answer-v0.1.json` describes the frozen positive and negative mutation set. Test code applies those mutations to a separately constructed valid capsule and checks the expected stable error/disposition. The validator does not generate the expected outcomes.

## Canonicalization

Canonical bytes sort stage receipts by DJCC stage order; sort input bindings by consumer slot, producer stage and producer receipt; and sort reason/unknown/conflict codes. The capsule commitment is domain-separated SHA-256 over those bytes. Any change to a bound receipt, schema, policy, scope, frontier, plan revision or outcome changes the commitment.

## Claim ceiling

The reference can establish only structural composition under this profile. It does not authenticate producers or source bytes, prove complete source inventories, verify the causal-log head, prove statistical/physical independence, verify a professional credential or legal authority, establish water-quality compliance, prove physical repair, or establish field effectiveness. Missing evidence remains NotEvaluated or Indeterminate.

## Profile-specific partial-order semantics

The DBC keeps three references distinct:

1. `CausalEventFrontier` — an accepted per-stream event cut. `Valid` means the selected stream prefixes and causal closure satisfy the frontier owner's contract; it is not source-universe completeness.
2. `ProgressAntichain` — the #6804 progress disposition, which may be `StableRelativeToDeclaredProgress` or `Provisional`. Its `source_universe_completeness` is fixed to `NotEvaluated`, matching #6804's claim ceiling.
3. `OwnerUniverse` — a separate receipt and independent completeness claim. Only this receipt can establish whether the declared source/owner universe is complete.

A stable progress antichain therefore cannot by itself establish completeness, and a structurally valid event frontier cannot establish that no source was omitted. The full-profile capsule requires all three bindings to make a positive completeness claim.

The supported DJCC profile is deliberately limited to `mycelix.djcc.full-evidence-execution.v0.1`. It does not claim independent corroboration; the independence dimension remains `NotEvaluated` unless a future separately specified profile includes authenticated failure-domain lineage.

## DJCC profile source binding

The stage graph and root-claim ownership in this oracle mirror the proposed `mycelix.djcc.full-evidence-execution.v0.1` profile from DJCC PR #6822, pinned for review to source commit `bf80359e9eb564a246da7cedb7d1476fb968d8f0`, file `tools/economic/evidence-execution-composition/DJCC_PROFILES_V0_1.json`, Git blob `36945535470677ef9ec8f06b9f68ff8bc457b7d3` (profile file SHA-256 `f05c2d0f280b60c75240f9933f89ba5376ac9a7250f36a3832c14c679bafe902`). That profile's manifest status is `proposed-structural-profiles-not-yet-authoritative`; this oracle mirrors its proposed contract but does not promote it to an authoritative policy or standard.

The full-evidence profile requires authentication, authority, completeness, effect and truth. Its root claim owners are verifier frontier/source trust for authentication; authorization for authority; verifier frontier/source trust/semantic projection/query for completeness; execution/outcome for effect; and query/policy/outcome for truth. The profile does not define independence as a required root claim, so this DBC profile deliberately leaves independence `NotEvaluated`. A positive independence claim belongs to the separate DJCC independent-corroboration profile and still requires #6826's authenticated failure-domain lineage, not merely a structurally disjoint graph.
