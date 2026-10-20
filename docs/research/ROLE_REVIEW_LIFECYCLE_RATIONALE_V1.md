# Role-review lifecycle rationale

## Purpose
This note records why the role-concentration theorem was refined from a permanent reviewer lock to an active-review lifecycle.

Intended boundary:
active review -> reviewer must remain role-disjoint
closed review -> historical evidence remains; reviewer may later reacquire a qualification role

This avoids turning separation of duties into a permanent incapacity rule.

## Design basis
NIST's 2026 software-agent identity/authorization work emphasizes explicit identity, authorization, auditing, and non-repudiation. NIST SP 800-53 AC-5 treats separation of duties as a control that reduces abuse risk.

The Mycelix translation is narrower:
qualification-role concentration != unlawful conduct
qualification-role concentration -> explicit conflict finding
full role concentration -> independent review
independent review -> distinct reviewer + role-disjoint reviewer
active reviewer role drift -> rejected
review closure -> role lock released

OECD's 2026 AI-market work separately identifies contestability, switching barriers, gatekeeping, infrastructure concentration, vertical integration, and interoperability as structural concerns. Those belong to the separate de-facto-concentration tranche and are intentionally not collapsed into this theorem.

## Formal evidence model

### TLA+
The lifecycle model records current role holders, conflict findings, active external reviewers, historical reviewers, and logical time.
Key properties: TypeOK; RoleConcentrationRequiresFinding; FullControlRequiresIndependentExternalReview; ReviewerRoleDisjointness; ReviewHistoryPreserved.

### Alloy
The structural model checks concentration with an explicit finding, full control with an independent reviewer, closed-review role reacquisition, self-review rejection, same-role reviewer rejection, and active reviewer-role drift rejection.

### Executable reference
The bounded oracle provides a canonical depth-6 no-violation exploration, negative counterexamples for missing findings/review/self-review/same-role review/reviewer-role drift, and a positive review -> close -> later role reacquisition witness.

## Claim ceiling
This note does not establish legal independence, institutional legitimacy, moral status, personhood, sovereignty, or competition-law compliance.
It does not require a universal separation-of-duties rule for every organization.
It is a narrow qualification-process theorem for active review events.

## Sources / research trace
- NIST (2026), software-agent identity and authority work.
- NIST SP 800-53 Rev. 5, AC-5 Separation of Duties.
- OECD (2026), Artificial Intelligence markets.
- Mycelix #4530, de facto sovereignty, accumulation and anti-entrenchment.
- Mycelix QUAL-001 / #866, separation of theorem subject from qualification-verifier authority.
