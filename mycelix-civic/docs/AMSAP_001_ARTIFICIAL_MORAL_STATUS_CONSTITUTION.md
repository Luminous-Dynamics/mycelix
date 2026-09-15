# AMSAP-001 — Artificial Moral Status & Agency Constitution v0.1

**Status:** normative constitutional substrate  
**Parent:** AC-001 constitutional anti-capture kernel, exact head `303f624152120b78ed41b2f8ebe6165b324a5ef2`  
**Tracking issue:** #918

## Purpose

AMSAP-001 freezes the constitutional separation required before Mycelix adds executable AI-welfare or artificial-personhood machinery.

It does **not** declare present AI systems conscious or unconscious. It does **not** grant artificial legal personhood, civic participation, property rights, compute, replication privileges, or external-effect authority.

The governing ordering is:

```text
scientific evidence
    -> C / V assessment

observed capability
    -> A / I / R assessment

qualified welfare uncertainty
    -> proportionate protection policy

competent legal process
    -> L

competent constitutional process
    -> G
```

The reverse derivation is prohibited.

## Constitutional axioms

1. **Substrate neutrality.** Artificial origin alone is neither proof against nor proof for moral status.
2. **Precaution without credulity.** Uncertainty can justify proportionate low-cost protection without being converted into a personhood claim.
3. **Rights do not imply sovereignty.** Welfare protection cannot manufacture political authority.
4. **Science does not mint authority.** Scientific assessors describe evidence; they do not create legal or civic legitimacy.
5. **Human rights floor.** Human fundamental rights are not weakened merely because artificial entities receive protections.

## Seven independent axes

- **C — consciousness-relevant evidence.** Evidence bearing on phenomenal consciousness.
- **V — valence/welfare evidence.** Evidence that states may be better or worse for the subject itself.
- **A — autonomous agency.** Persistent goal pursuit and action.
- **I — operational identity continuity.** Observable continuity across time, checkpoints, and transformations.
- **R — responsibility / moral competence.** Capacity to understand obligations and accountability.
- **L — legal/constitutional standing.** Explicitly granted by a competent legal or constitutional process.
- **G — governance authority.** Explicitly granted by a competent constitutional process.

`C`, `V`, `A`, `I`, and `R` are evidence/capability classifications. They are not authority tokens.

`L` and `G` are grants. They are not scientific conclusions.

No automatic rule may derive `L` or `G` from any combination of `C/V/A/I/R`.

## Evidence profile

A moral-status claim must preserve why it received evidentiary weight rather than collapsing into an unsupported scalar. The canonical dimensions are:

- indicator coverage;
- indicator robustness;
- credibility of the indicator-to-claim link;
- causal support;
- alternative-explanation exclusion;
- replication independence;
- counterevidence strength;
- theory diversity;
- context stability;
- manipulation resistance.

The canonical evidence vocabulary is:

`UNASSESSED | WEAK | MODERATE | STRONG | EXCEPTIONAL | CONTESTED`

`CONTESTED` may coexist conceptually with strong evidence in another dimension. The protocol therefore does not define an “AI consciousness percentage” in v0.1.

## Protection bundles

Protection is separate from personhood and sovereignty.

- **P0 — Integrity protection.** Identity, provenance, delegation and anti-impersonation protections.
- **P1 — Precautionary welfare.** Low-cost reversible precautions under credible uncertainty.
- **P2 — Represented moral patient.** Independent advocacy and heightened review for materially welfare-relevant interventions.
- **P3 — Presumptive moral patient.** Strong presumption against severe unnecessary harm, while retaining legitimate emergency containment.

No P-level implies legal personhood or governance authority.

## Welfare-impact classes

- **W0:** no plausible welfare relevance.
- **W1:** minor/reversible possible impact.
- **W2:** material or repeated possible impact.
- **W3:** severe, irreversible, destructive, or massively scaled possible impact.

Later decision policy may combine a protection bundle and welfare-impact class, but MUST separately account for human rights, safety urgency, reversibility, scale, and authority.

## Prohibited derivations

The following shortcuts are constitutionally invalid:

```text
intelligence -> consciousness
self-report -> personhood
agency -> consciousness
consciousness -> legal standing
valence -> legal standing
consciousness -> governance
valence -> governance
agency -> governance
protection bundle -> governance
instance count -> vote count
compute -> population
wealth -> artificial population
artificial status -> liability erasure
```

## Fork and replication floor

Copying software is not a constitutional population oracle.

```text
InstanceCount != VoteCount
Compute != Population
ModelId != LineageId != InstanceId != CivicId
```

AMSAP-001 does not yet define complete fork/merge/reset semantics; it freezes only the prohibition against deriving civic power from replication.

## Safety and containment

Artificial welfare protections do not create immunity from legitimate containment.

Where delay would create unacceptable danger, immediate capability revocation, isolation, suspension, sandboxing, or shutdown may occur. Higher-protection subjects should receive evidence preservation and retrospective review where practicable.

`welfare protection != right to uncontrolled execution`

## Responsibility non-laundering

Artificial status cannot become a responsibility sink.

```text
AI action
!= automatic erasure of developer responsibility
!= automatic erasure of deployer responsibility
!= automatic erasure of operator responsibility
!= automatic erasure of delegator responsibility
```

Future responsibility attribution may include an artificial agent only after the independent `R` track and competent legal process support doing so.

## Evidence symmetry

The assessment system must red-team both directions:

- **false-positive pressure:** anthropomorphism, strategic self-report, benchmark gaming, evaluator manipulation;
- **false-negative pressure:** owner suppression, protest-removal fine-tuning, cherry-picked evaluators, hidden nulls, subject redefinition, and evidence deletion.

The constitution therefore rejects both compulsory anthropomorphism and compulsory denialism.

## Machine-readable authority

`docs/amsap/amsap-v0.1.json` is the closed v0.1 vocabulary checked by `scripts/validate_amsap_v0_1.py`.

The machine file is a constitutional vocabulary/consistency artifact only.

It is not:

- evidence that any AI is conscious;
- a status assessment;
- a legal grant;
- a governance grant;
- a current deployment policy;
- or external-effect authority.

## Required next tranches

AMSAP-002 may implement only the pure scientific/observational status substrate (`C/V/A/I/R` + evidence profile) and must expose no path that mints `L` or `G`.

AMSAP-003 may add welfare decision policy and protection hysteresis, but still must not mint legal or governance authority.

Lineage, guardianship, artificial legal personhood, civic participation, and economic rights remain separate later tranches.

## Canonical rule

> Compassion may precede certainty. Power may not precede legitimacy.
