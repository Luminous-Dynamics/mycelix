# Relational capability Rust reference — comparison and qualification crosswalk

**Artifact:** `research/relational-capability-containment-v0.1/`  
**Related formal candidate:** [SOV-AI-AUTH-v17 / PR #4867](https://github.com/Luminous-Dynamics/mycelix/pull/4867)  
**Related owners:** [#4852](https://github.com/Luminous-Dynamics/mycelix/issues/4852), [#6302](https://github.com/Luminous-Dynamics/mycelix/issues/6302), [Xenia #517](https://github.com/Luminous-Dynamics/xenia-peer/issues/517)

## Purpose

The Rust crate is an independent, dependency-free implementation candidate for relational capability containment. It provides a Rust-side exact tuple model and executable property tests that can later be compared to the existing TLA+/Alloy/Python formal candidate.

It does **not** duplicate or replace PR #4867, ROS-005, the Holochain substrate, or the Xenia runtime. It is a second implementation boundary for cross-checking the theorem, not a claim that two different source models are already equivalent.

## Model comparison

| Semantic dimension | SOV-AI-AUTH-v17 / PR #4867 | Rust reference v0.1 |
|---|---|---|
| Operation/action | operation enum/value | action scope value |
| Target/resource | target | resource |
| Audience | audience | audience |
| Data/control direction | not a separate tuple field in the payment profile | explicit exact Direction enum |
| Purpose | not a separate tuple field in the payment profile | exact/wildcard purpose |
| Data/argument class | argument_class | data_class |
| Effect mode | not a separate tuple field in the payment profile | Propose / Execute subsumption |
| Operation identity | consequential tuple semantics | opaque operation_binding field |
| Currency/unit | currency enum | explicit exact quantitative-limit dimension |
| Numeric ceiling | numeric_bound | u64 maximum attached to a dimension |
| Wildcards | parent-side categorical wildcards | parent-side Any values for scope fields |
| Alias semantics | target alias normalization (e.g. acct-alice ↔ alice) | no alias normalization |
| Set containment | each child tuple requires a parent tuple witness | each child tuple requires a parent tuple witness |
| Formal model | TLA+ and Alloy plus Python reference/control matrix | Rust reference with deterministic unit/property tests |
| Canonical identity | candidate's own schema and identity controls | separate schema-v1 canonical bytes; not byte-compatible |

These are related but not identical schemas. In particular, a payment currency is not automatically interchangeable with a Xenia data class, direction or purpose. The Rust limit dimension must match exactly; no implicit unit conversion is performed.

## Shared theorem

Both models are intended to preserve the same central property:

`ChildCapabilitySet ⊆ ParentCapabilitySet`

Each concrete child capability must have a *complete parent tuple witness*. It is not sufficient for each column to appear somewhere in a parent marginal projection. This rejects unauthorized Cartesian recombinations.

The Rust model's `is_subset_of` checks exact relational witness subsumption across action, resource, direction, audience, purpose, data class, mode, operation binding and quantitative limit. The quantitative dimension must match and the child's maximum must not exceed its witness parent's maximum.

For wildcard scope values, a parent Any subsumes an exact child; an Any child is not subsumed by an exact parent. Direction is deliberately non-wildcard. Propose is contained by Execute, not vice versa.

## Differences and known gaps

1. **Alias normalization:** PR #4867 explicitly models selected target aliases as semantically equivalent. Rust v0.1 rejects no alias but treats distinct strings as distinct values. It does not claim normalization equivalence.
2. **Currency and units:** PR #4867 has a payment currency field and numeric bound. Rust v0.1 uses a dimension-tagged numeric ceiling and requires exact dimension equality. Currency/value conversion is not attempted.
3. **Purpose and direction:** Rust adds dimensions needed by remote access; they are not direct equivalents of a payment argument class.
4. **Operation digest:** Rust's operation_binding is caller-supplied opaque data. It is not signed, verified, canonicalized from a concrete request, or proven to equal an actual OS/provider effect.
5. **Authority/currentness:** neither this finite Rust crate nor the pure relational reference alone authenticates principal, issuer, parent records, trusted time, current authority epoch, revocation propagation, DHT visibility or Holochain call capability.
6. **Budgets:** a per-tuple numeric maximum is a static upper bound. It is not durable consumption, sibling aggregation, rolling-window rate enforcement, concurrency control or restart-safe one-shot use.
7. **Formal verification:** Rust tests do not substitute for independent TLA+/Alloy execution. Conversely, a formal model does not prove that a production adapter or operating-system effect path implements its semantics.

## Proposed differential qualification sequence

1. Freeze a shared *exact semantics* fixture subset with both model inputs, schema versions, and a separately authored expected oracle.
2. Map only fields with an explicit total mapping; mark unsupported fields as UNSUPPORTED rather than filling them with invented defaults.
3. Run positive, negative and metamorphic controls:
   - exact parent tuple and strict child subset;
   - one-field mutations for action, target/resource, audience, direction, purpose, class, mode, operation binding and numeric bound;
   - Cartesian recombination with all marginals individually present;
   - parent wildcard vs exact child and child wildcard vs exact parent;
   - increased numeric ceiling and mismatched unit;
   - duplicate tuple, reordered tuple list, unknown schema version and malformed scope atoms;
   - alias-normalization cases only after the Rust side has a versioned explicit normalization profile.
4. Compare dispositions and minimized counterexamples, not merely printed PASS labels.
5. Bind exact source SHAs, tool versions, fixture digests, model bounds and output artifacts. A skipped or queued formal job is NOT_RUN, not PASS.
6. Only after both models agree on the shared frozen corpus should the two model owners consider a normalized v2 scope contract. Preserve the two v1 profiles; do not silently reinterpret existing bytes.

## Xenia mapping

Xenia's M1 currently has eight statically paired permission/action/direction lanes in [PR #509](https://github.com/Luminous-Dynamics/xenia-peer/pull/509). A future complete effect tuple could bind:

`(action_id, direction, host/resource, audience/session_fingerprint, purpose, data_class, mode, operation_digest, authority_epoch, quantitative_limit)`.

The current Xenia runtime does not consume this Rust crate or ROS-005. The mapping is a prospective integration contract for [Xenia #517](https://github.com/Luminous-Dynamics/xenia-peer/issues/517), not a production integration claim.

## Claim ceiling

This artifact supports only the semantics implemented in this finite Rust reference and tests that are actually run on its exact source head. It does not demonstrate general authorization security, cryptographic authenticity, complete mediation, live revocation, Holochain compatibility, or external-effect correctness.
