# Counterfactual Repair Certificate V1

**Status:** draft reference contract.  
**Implementation:** `craft-counterfactual-repair-dissent-v1`.  
**Related benchmark:** Evidence Assurance Metamorphic V1, MR-017.

## Purpose

An open decision should expose the exact material blockers and the lawful, owner-qualified evidence or review requirements needed before the decision can be reassessed. A system that only says “blocked” is not sufficiently useful; a system that invents a workaround, silently waives an element, or converts a recommendation into permission is unsafe.

The certificate is an immutable, read-only explanation of an owner-proposed path to a fresh owner decision. It is not an execution plan in the operational-control sense.

## Inputs and composition

A packet contains:
- an exact decision revision and use-context digest;
- the authoritative decision disposition and its digest;
- owner evaluation references with exact evaluator version, decision/context binding, and a material-coverage receipt;
- material blockers reported by those owner evaluators;
- exact evidence/authority requirements and blocker dependencies;
- guidance steps proposed by the owner evaluators, each bound to exact context, the blockers and requirements it addresses, and the owner that must reassess any new material.

Supported owner classes are Need Intake, Claim Support, Decision Closure, Evidence Lineage, Impact Qualification, and a domain owner. This crate composes owner outputs; it does not recreate their policy.

## Required invariants

1. **Exact decision binding.** Every owner evaluation and step is bound to the exact decision ID, revision, decision digest and context digest.
2. **Complete owner coverage.** Each owner receipt must attest complete material-element coverage. The host must authenticate and validate the meaning of that receipt; a boolean-like field alone is not proof.
3. **Blocker coverage.** Every reported blocker has at least one owner-proposed guidance step.
4. **Requirement coverage.** Every evidence and authority requirement on each blocker is addressed by a step that references the exact same typed requirement or authority artifact. A step cannot substitute a provider assertion for an independent observation, or a different authority policy for the required one.
5. **Dependency integrity.** Blocker and step dependencies must resolve and be acyclic.
6. **No duplicate steps.** Semantically duplicate steps are rejected. One owner-proposed step may address multiple blockers only when the requirement is exactly the same for each.
7. **Disposition preservation.** The output carries the exact decision disposition and digest unchanged.
8. **No authority side effect.** Guidance steps are declarative requirements only. The crate exposes no dispatch, authorization, closure, payment, procurement, permit, or execution interface.

## Limits and trust boundary

The crate validates structural consistency and exact references. It does **not**:
- authenticate signatures, issuers, reviewers, delegated authority or revocation;
- determine whether an evidence source is truthful, independent, qualified, current or legally admissible;
- decide which material elements are required for a domain conclusion;
- establish that an owner's material-coverage attestation is true;
- execute a next step, grant an approval, extend a lease, or close a decision;
- compute a globally minimum hitting set or prove that an owner-proposed plan is the unique mathematically minimal plan.

The current contract prevents omitted reported blockers, omitted exact requirements, cycles and duplicate semantic steps. A future optimizer may reduce the number of separately requested steps by grouping genuinely identical requirements, but it must preserve owner qualification, dependency ordering and exact coverage. It must be evaluated against MR-017 and must never make an unsafe substitute appear admissible.

## Required host behavior

The host must authenticate every owner evaluation and coverage receipt; verify exact artifact bytes/digests and revocation/currentness; confirm that each requirement is appropriate for the jurisdiction, population, asset and use context; and present the certificate as advice. Any new evidence or authority decision returns to its owning evaluator. The host must not interpret a completed step as closure: the owner must re-evaluate, and the original authority must make any subsequent decision.

## Acceptance tests

The crate covers exact decision/context binding, incomplete owner coverage, missing and unknown blockers, exact evidence-kind and authority matching, full requirement coverage, cyclic dependencies, duplicate semantic steps, read-only disposition preservation, and the difference between evidence acquisition and an authority decision. CI compile/test/Clippy results are tracked independently from integrated evaluator qualification.
