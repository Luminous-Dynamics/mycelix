# mycelix-djcc-verifier

A dependency-free Rust structural candidate verifier for DJCC V0.1.

## Run

From the repository root:

```sh
cargo +1.96.0 fmt --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml -- --check
cargo +1.96.0 test --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml --all-targets
cargo +1.96.0 clippy --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml --all-targets -- -D warnings
```

## What the candidate checks

- Eight required stage identifiers and the fixed allowlisted dependency graph.
- Exact selected profile reference/digest and one common scope digest/schema binding.
- Producer owner/version identity and source-commit syntax.
- Exact named consumer input/digest/schema binding to producer output.
- Exact profile-owned dependency parent set and exact required claim set on every edge; a certificate cannot weaken a requirement by omitting a dimension.
- Canonical ordering of dependency edges, required claim dimensions, input names and reason codes.
- Stage output/disposition consistency.
- Profile-limited `NotApplicable` stages, with all six claim dimensions explicitly NotApplicable and no profile edges consumed.
- Profile-owned root claim requirements: each established root dimension is supported by the exact owner-stage set recorded in the selected profile.
- A stage may remain overall Indeterminate while exposing one established claim dimension. A consumer may rely on that dimension only when its edge names it; a query that needs Completeness cannot use Authentication as a substitute.
- A NotEvaluated stage cannot claim that it evaluated any dimension; a Rejected stage can retain a separately established authentication claim while rejecting the factual proposition.

## Fixed full-profile edge requirements

`DJCC_PROFILES_V0_1.json` is the external profile registry. The full evidence-to-outcome profile pins these edge claim requirements:

| Consumer | Producer | Required upstream claims |
|---|---|---|
| Source trust | Verifier frontier | Authentication, Completeness |
| Semantic projection | Verifier frontier | Authentication |
| Semantic projection | Source trust | Authentication |
| Query | Verifier frontier | Authentication, Completeness |
| Query | Source trust | Authentication, Completeness |
| Query | Semantic projection | Authentication, Completeness |
| Policy | Query | Authentication, Completeness, Truth |
| Authorization | Verifier frontier | Authentication |
| Authorization | Policy | Authentication, Truth |
| Execution | Authorization | Authentication, Authority |
| Outcome | Source trust | Authentication |
| Outcome | Semantic projection | Completeness |
| Outcome | Execution | Effect |

This expresses a useful distinction: a correctly signed and registered source statement may be parsed while issuer participation completeness remains Indeterminate; a query that asserts complete coverage must wait for Completeness.

The profile separately assigns root claim dimensions to their responsible stages. Full evidence-to-outcome requires authentication, authority, completeness, effect and truth; the independent-corroboration profile additionally requires independence. The historical-decision-only profile explicitly makes execution and outcome NotApplicable and does not claim an effect. The third profile uses explicit NotApplicable stages for execution/outcome and does not assert a physical outcome.

## Shared Go/Rust edge KAT

`DJCC_STAGE_EDGE_V1_KAT.tsv` fixes the edge-preimage encoding:
- domain separator `MYCELIX-DJCC-STAGE-EDGE-V1`;
- producer stage, consumer stage, consumer input and producer output-schema strings encoded as UTF-8 with big-endian u32 byte-length prefixes;
- a big-endian u32 count of required claim dimensions, followed by their canonical strings;
- the 32-byte producer output digest appended raw.

Go and Rust independently encode the same four known-answer vectors, including a non-ASCII input name and a dependency edge requiring the independence claim. This is edge-level byte parity only, not full-certificate canonicalization or signature verification.

## Not verified by this crate

The crate does not authenticate stage producers, verify signatures or trusted time, reconstruct owner universes, authenticate transparency tree heads, prove inclusion/consistency, parse the JSON certificate, evaluate policy semantics, establish legal authority, execute effects, verify IVC proofs, or adjudicate physical outcomes.

This is a structural candidate, not a cryptographic trust implementation or production assurance service.


## Failure-domain lineage and resilience submodule

The `failure_domain` module checks a declared lineage DAG for canonical identifiers, required failure-domain class coverage, missing parents, cycles, unreachable nodes, shared material ancestors and explicitly classified non-material shared classes. It returns one of:

- `StructurallyDisjointWithinDeclaredGraph`
- `CommonCauseDetected`
- `LineageCoverageIncomplete`

The assessment represents common causes as **shared-lineage groups**: one record per shared domain ID, with its class, all affected evidence-channel IDs, and its classification (material, explicitly non-material or unclassified). This avoids materializing an O(channel-count squared) empty pair matrix; cost is proportional to total lineage references plus the actual shared groups.

A separate deterministic failure-scenario evaluator takes a named independence profile and scenarios that identify exact domain IDs assumed unavailable. It reports affected channels, surviving channels, the survivor floor, and shared-lineage groups that remain among the survivors. Its output separately carries the profile's structural independence disposition: a passing survivor floor never promotes incomplete lineage or erases common causes. It does not estimate failure probabilities, prove the scenario set exhaustive, or imply that surviving channels are mutually independent. This creates a decision-relevant resilience envelope rather than a misleading scalar independence score.

A shared domain class omitted from both the material and explicitly non-material profile sets remains `LineageCoverageIncomplete`; no unlisted shared class is silently treated as harmless. A structurally disjoint graph is not a trusted independence claim: the verifier does not authenticate who supplied the graph, prove that its lineage is complete, or decide whether the declared failure model covers all relevant common causes. Those remain source-trust/profile obligations. Tests include disjoint channels, shared upstream feeds, missing key-custody lineage, material-but-not-required shared infrastructure, explicitly permitted non-material infrastructure, unclassified shared domains, common-cause quorum failure, survivor floors, cycles, missing parents and canonical-order mutations.
