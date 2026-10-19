# Mycelix Evidence Manifest v1

Status: Draft evidence specification  
Tracking: #882 (`MYC-EVID-001`)  
Purpose: Keep technical, security, commercial and capital claims machine-current and provenance-preserving.

## 1. Core rule

Mycelix should not allow manually copied status prose to become stronger than the underlying evidence.

```text
marketing statement
!= technical qualification
!= security assurance
!= customer outcome
!= financing event
```

Every reusable claim must name its evidence plane, authority class and source.

## 2. Four evidence planes

### 2.1 Technical Evidence

Tracks exact theorem/profile qualification.

Required fields should include, where applicable:

```text
theorem/profile id
subject commit/tree
verifier profile/head
gate/workflow id
run + attempt
disposition
receipt/evidence digest
qualified_at
dependencies
current/superseded state
nonclaims
```

Allowed dispositions should preserve meaningful distinctions such as:

```text
PASS
FAIL
PENDING
INFRASTRUCTURE_NONEXECUTION
SUPERSEDED
NOT_ASSESSED
UNSUPPORTED
```

Rules:

```text
queued != PASS
draft != PASS
mergeable != PASS
ancestor PASS != child PASS
planned tests != PASS
```

### 2.2 Security Evidence

Tracks assurance separately from theorem correctness.

Candidate fields:

```text
bounded product/profile
threat-model revision
review/audit provider class
review scope
assessment date
covered subject/profile
open critical/high findings
remediation evidence
residual risks
assurance expiry/currentness
```

No finding observed is not equivalent to proven safe.

### 2.3 Commercial Evidence

Tracks deployment and economic facts.

Candidate fields:

```text
design-partner/deployment id
disclosure class
paid/unpaid state
product/profile + maturity level
baseline period
post-deployment period
time to deploy
governed identities/agents/actions
receipt volume
customer-value metrics
renewal/expansion state
revenue/ARR where appropriate
evidence authority class
```

A customer using the product does not upgrade technical or security qualification.

### 2.4 Capital Evidence

Tracks financing provenance and associated rights at the permitted disclosure level.

Candidate fields:

```text
source class
recipient entity/project
instrument class
effective date/interval
governance/control-right profile
use restrictions
disclosure class
MYC-CAP profile revision
```

A financing event does not prove product-market fit, safety, technical correctness or wise capital allocation.

## 3. Claim authority classes

Every externally reusable claim should be classified explicitly.

Suggested v1 vocabulary:

```text
Qualified
ExternallyAssured
Observed
CustomerAttested
IndependentlyVerifiedOutcome
Projected
Aspirational
Unsupported
Stale
```

These classes are not a total ordering.

For example, a customer-attested ROI number may be commercially important but should not be presented as an independently verified outcome.

`Projected` requires a named model/profile and explicit assumptions.

`Aspirational` may describe strategy but must not appear in a current-capabilities table as if implemented.

## 4. Canonical source and projections

Prefer one canonical machine-readable manifest with deterministic projections.

```text
canonical manifest
   |
   +-> generated Markdown status
   +-> due-diligence export
   +-> public website/status projection
   +-> pitch/board metrics snapshot
```

Projections may redact confidential fields but must not strengthen the underlying claim.

No projection may convert `Unknown`, `NotAssessed`, `Pending`, `Failed` or `Stale` into a positive state.

## 5. Currentness model

Currentness must be explicit, not inferred from whichever branch or file happens to be newest.

Each evidence family should designate the authoritative subject/profile lineage.

Examples:

```text
claim subject != designated current subject
    -> Historical or Stale

new authoritative run supersedes prior current run
    -> old run remains historical evidence

security assessment predates material profile change
    -> assessment no longer proves current-profile assurance
```

Historical evidence is append-only. Supersession does not rewrite an old failure into success.

## 6. Motivating AGENT example

The manifest should prevent a roadmap from manually retaining an older exact head or run state after a newer designated AGENT subject exists.

The correct workflow is:

```text
designated AGENT profile
    -> exact designated subject
    -> exact authoritative run/receipt
    -> generated status projection
```

Human-authored prose may explain why a run failed, but it must not override the machine disposition.

## 7. Human-readable status table

A generated summary should preserve distinct axes.

| Capability/Profile | Maturity | Technical | Security | Commercial | Current? |
|---|---|---|---|---|---|
| Example | R1 | PASS | Not assessed | No pilot | Yes |

The table must not collapse `Not assessed` into either pass or fail.

## 8. Commercial maturity integration

Consume the maturity vocabulary from `MYCELIX_TRUST_FABRIC_V1.md`:

```text
R0 Research
R1 Qualified Core
R2 Simulation Integration
R3 Controlled Pilot
R4 Bounded Production
R5 Federated Production
R6 Infrastructure / Sovereign
```

Maturity level is derived from named gates, not manually asserted.

Example:

```text
R4 requires
  technical production path qualified
  + required external assurance current
  + bounded deployment evidence
  + operational rollback/reconciliation controls
```

## 9. Investor/customer readiness gates

The manifest may compute readiness predicates, but not fundraising certainty.

Candidate predicates:

- IP/licensing coherence complete;
- designated commercial product boundary exists;
- required technical lineage qualified;
- external assurance current;
- paid design-partner evidence exists;
- deployment cost/time measured;
- customer value measured;
- repeatability demonstrated;
- interoperability/federation evidence available.

A readiness predicate should report its missing inputs explicitly.

## 10. Privacy and disclosure

Evidence may have disclosure classes such as:

```text
Public
PartnerConfidential
InvestorConfidential
SecurityRestricted
Internal
```

The public manifest may contain commitments/references rather than customer names or sensitive details.

A redacted projection must be monotonic in strength: removing detail cannot make a claim appear better supported than the full record.

## 11. Initial schema sketch

A future canonical representation may have the following top-level shape:

```text
manifest_version
profile_registry
technical_evidence[]
security_evidence[]
commercial_evidence[]
capital_evidence[]
claim_registry[]
maturity_projections[]
readiness_projections[]
```

Authority-bearing structures should reject unknown fields unless the profile explicitly allows extensibility.

Canonicalization and commitment format must be versioned before hashes are treated as stable evidence identities.

## 12. Required regressions

The first executable implementation should prove at minimum:

1. queued/pending is not PASS;
2. failed is not PASS;
3. superseded PASS remains historical rather than current;
4. newer designated subject makes old current-status projection stale;
5. an ancestor PASS cannot qualify a child;
6. customer success cannot upgrade technical state;
7. technical PASS cannot fabricate external security assurance;
8. funding cannot upgrade commercial or technical state;
9. redaction cannot strengthen a claim;
10. missing evidence produces explicit `NotAssessed`/`Unsupported` rather than a guessed positive value;
11. canonical input produces deterministic projection;
12. unsupported schema/profile revisions fail closed.

## 13. Evidence-backed ROI

Customer-value claims should retain baseline, intervention and outcome identity.

Conceptually:

```text
baseline
+ deployment/profile
+ observation interval
+ metric definition
+ outcome
+ evidence authority
-> reusable commercial claim
```

This allows a claim such as audit-time reduction to remain tied to the exact measurement definition rather than becoming free-floating marketing copy.

## 14. Nonclaims

The manifest does not prove:

- valuation;
- fundraising success;
- legal/securities compliance;
- correctness of self-reported customer data;
- absence of unknown vulnerabilities;
- product-market fit;
- wisdom of a technical or financial decision.

Its purpose is narrower and valuable: **make Mycelix commercial claims obey the same provenance discipline as Mycelix technical claims.**
