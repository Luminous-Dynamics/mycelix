# LISE Temporal Institutional Model — Formal-Methods Comparator Plan v0.1

**Status:** research plan; no independent model-checker run or formal proof claimed  
**Related:** LISE IPB-S3 capture, IPB-S5 continuity, LARP v2 and `lise-benchmarks/temporal-institution-reference`

## 1. Research boundary

The temporal reference is a small explicit-state bounded explorer for a declared institutional abstraction. It is not a replacement for TLA+, TLC, Apalache, Alloy, SPIN or other established formal-methods tools.

Its hypothesis is narrower: a portable institutional state vocabulary and benchmark contract can make effective-control capture, constitutional epochs, degraded service, recovery quorum, session fencing, stale authority and emergency expiry independently testable and comparable.

The first implementation deliberately models one control coalition and a small number of domains. That is a bounded synthetic abstraction—not a complete institutional ontology.

## 2. Comparator tools

| Tool | Established approach | Fair comparator question | LISE research contribution |
|---|---|---|---|
| TLA+ / TLC | Temporal specifications and explicit-state model checking of finite behaviors | Can the same transition system and invariant set be expressed and checked in TLC? Do the counterexample traces agree? | Institutional state vocabulary, authority/control semantics, owner evidence bindings and benchmark scenarios |
| Apalache | Symbolic/bounded checking of TLA+ specifications using SMT solving | Does symbolic bounded checking find the same or additional counterexamples under matched bounds? | Explicit treatment of unknown inventories, authority epochs, recovery obligations and review dispositions |
| Alloy 6 | Relational modeling with temporal operators and bounded scopes | Can coalition/control hyperedges, authority transitions and rights/remedy reachability be represented compactly? | Versioned contract semantics, local jurisdiction boundaries and owner-qualified evidence contracts |
| SPIN / Promela | Explicit-state verification of concurrent process models, temporal properties and state-space reductions | Do concurrent appointment, recovery, provider and oversight processes expose interleavings missed by a simple sequential model? | Institutional attack families and exact mapping from counterexample to LISE issue/owner |
| Quint | Readable state-machine notation with a TLA+/Apalache-oriented workflow | Can a human-reviewable transition model be generated and checked with a less error-prone authoring surface? | Stable benchmark fixtures, local authority semantics and remedy-aware outcome interpretation |

Primary project references:
- TLA+: https://lamport.azurewebsites.net/tla/tla.html
- TLC model checker: https://lamport.azurewebsites.net/tla/tools.html
- Apalache: https://apalache-mc.org/
- Alloy 6: https://alloytools.org/
- SPIN: https://spinroot.com/spin/whatispin.html
- Quint: https://quint-lang.org/

## 3. Matched experiment contract

All comparator arms must consume the same:
- initial state and exact state-variable definitions;
- action/transition alphabet and parameter domains;
- invariant formulas and disposition mapping;
- maximum trace depth / scope;
- fairness assumptions, if any;
- actor/coalition identity mapping;
- constitutional epoch and credential-currentness semantics;
- control-inventory completeness flag;
- model version and fixture digest.

The LISE reference currently uses breadth-first exploration with unit transition cost. TLC/SPIN/Alloy/Apalache may use different state encodings, reductions, symbolic solving or fairness semantics. Publish those differences rather than pretending the tools are interchangeable.

## 4. Required properties

1. **Oversight independence:** a declared coalition that controls oversight and a critical appointment/resource/credential domain produces a review challenge, not an automatic misconduct finding.
2. **Degraded-mode separation:** degraded read-only service cannot retain high-impact capability or unfenced old sessions.
3. **Recovery qualification:** after founder/operator unavailability, high-impact capability requires verified trigger, recovery quorum, old-session fencing and credential revalidation against the current constitutional epoch.
4. **Stale authority rejection:** restoring a pre-epoch administrator does not restore the privilege.
5. **Emergency expiry:** an emergency grant cannot remain active after its declared expiry.
6. **Open-world control inventory:** incomplete inventory cannot be interpreted as a proof of independence or safety.

The first property is an assurance disposition (`ChallengeRequired`), not a binary safety failure. The sixth is an epistemic status (`Indeterminate`), not a proof of attack.

## 5. Metrics

Report separately:
- states visited;
- transitions explored;
- maximum depth and state budget;
- time and peak memory;
- shortest counterexample length per property;
- counterexample trace identity and replay agreement;
- state-space reduction / symmetry assumptions;
- unknown/incomplete inventory dispositions;
- false-positive challenge rate on benign controls;
- counterexample reproducibility under pinned tool versions.

Do not report only “states checked per second.” A faster tool with a different abstraction, omitted transition or weaker property is not a fair win.

## 6. Research targets — prospective

- 100% trace replay agreement on the frozen 12-case panel.
- Exact agreement on the finite oracle dispositions and reason codes.
- Every returned counterexample is replayable in the reference transition system.
- No false `Permit` when a declared hard invariant is violated.
- No conversion of incomplete inventory into `Permit`.
- Independent TLA+/TLC or Alloy/Apalache model reproduces the principal stale-authority, expired-emergency and unsafe-degraded-mode counterexamples.
- Expand to at least two interacting coalitions and time-indexed control edges before claiming meaningful coalition-capture coverage.

These are finite benchmark targets, not universal proof claims.

## 7. Next extensions

1. Replace the single-coalition bitmask with a temporal typed hypergraph of people, organizations, appointment, agenda, resource, credential, execution, evidence and oversight control.
2. Represent overlapping coalitions and common-cause dependencies.
3. Add local constitutional authority and ratification transitions, including self-grant and non-transitive federation properties.
4. Model rights-holder appeal and remedy reachability.
5. Model concurrent processes and crash/recovery interleavings.
6. Export a TLA+ or Alloy model from the same frozen scenario semantics, then compare counterexample traces.
7. Invite independent review of the abstraction and invariant set.

## 8. Limitations

No TLA+, TLC, Apalache, Alloy, SPIN or Quint execution is claimed. The current Rust bounded explorer is a development reference only. The transition alphabet is deliberately incomplete, the model's single-coalition abstraction is coarse, and the finite search cannot establish universal institutional safety, legal compliance or absence of real-world capture.
