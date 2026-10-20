# AC-001 — Constitutional Anti-Capture Kernel

Status: draft implementation tranche

## Purpose

AC-001 defines a deliberately small constitutional boundary for Mycelix Civic. It does not choose policies, elect leaders, determine guilt, rank citizens, or assign human value. It exists to make a narrow set of structural anti-capture properties fail closed before higher-level governance logic can rely on them.

The design principle is:

> Power may accumulate, but accountability, contestability, and exit must accumulate faster.

## Machine-enforced invariants in AC-001

1. **Equal fundamental civic standing.** Economic capital, token balances, reputation, or compute ownership cannot multiply fundamental civic power.
2. **Bounded delegated authority.** Delegated authority requires an identified subject, domain, scope, purpose, forward expiry, revocability, and an appeal path.
3. **AI non-sovereignty over rights.** Automated systems may advise on rights-affecting matters but cannot be the final rights-affecting decision maker.
4. **Due process for rights-affecting consequences.** Observations, anomaly signals, automated inference, or reputation cannot directly remove rights. Rights-affecting consequences require an adjudicated finding and an appeal path.
5. **No universal reputation score.** Reputation may be explicitly domain-scoped; a universal civic-worth score is rejected.
6. **Public-power provenance.** Exercise of public power requires an authority reference, a stated reason, and non-empty provenance references.
7. **Federated exit and replacement.** Federated governance configuration must permit exit and replacement of the governance provider.

## Constitutional invariants not yet fully encoded

The following remain normative requirements for subsequent tranches and adapters:

- transparency obligations increase with public power exercised;
- ordinary citizens retain a strong presumption of privacy;
- evidence provenance does not imply truth;
- capture metrics target institutions, processes, and concentrations rather than assigning political virtue to people;
- important institutional claims remain independently challengeable;
- downstream systems must not convert capture signals directly into punishment;
- no mandatory global identity database;
- no reputation gate for essential civic rights or services.

These are intentionally not encoded here until their semantics can be expressed without creating over-broad or ambiguous machine authority.

## Non-goals

AC-001 does not define:

- tax rates or wealth limits;
- electoral systems;
- procurement thresholds;
- lobbying legality;
- antitrust policy;
- ideology;
- substantive criminal or civil law;
- corruption guilt;
- merit, trustworthiness, or human worth;
- a universal capture score.

## Failure semantics

The kernel is fail-closed for the action types it knows how to validate. A rejected action returns one or more typed `AntiCaptureViolation` values. The kernel does not itself punish, remediate, or adjudicate. A caller must route failure into an appropriate human/institutional review path.

This separation is intentional:

`observation -> signal -> review -> adjudication -> consequence`

is allowed, while

`signal -> automatic rights loss`

is not.

## Placement

The implementation lives in `mycelix-civic/crates/civic-types/src/anti_capture.rs` so every Civic domain can share the same constitutional vocabulary without introducing a separate service or privileged coordinator zome.

## Qualification target

AC-001 is considered qualified only when:

- the pure unit tests pass;
- mutation or adversarial tests demonstrate that each prohibited path is rejected;
- downstream Civic code cannot bypass the kernel by silently mapping prohibited actions into an allowed variant;
- serialization round trips preserve the action and violation semantics;
- no test or production path assigns rights-affecting authority to an AI or capture detector.

The present tranche establishes the pure kernel and direct unit tests. Downstream integration and mutation coverage belong to the next qualification tranche rather than being silently claimed here.

## Next tranche

AC-002 should define the capture-observation vocabulary separately from AC-001: observations, hypotheses, measurements, uncertainty, provenance, alternative explanations, and review status. AC-002 must preserve the AC-001 rule that a capture signal is evidence for review, not an adjudicated finding.
