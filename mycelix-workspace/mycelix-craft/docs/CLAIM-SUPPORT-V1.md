# Craft Claim Support V1

- **Work item:** CRAFT-CLAIM-SUPPORT-001 (#6752)
- **Status:** Reference core; semantic evaluator and host integration pending
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## 1. Three separate propositions

A trustworthy proposal must not collapse these propositions:

1. **Provenance:** this claim points to this exact source version and passage.
2. **Semantic support:** the passage supports the claim as written, at its exact scope, time, units, strength and qualifiers.
3. **Truth and authority:** the source is reliable and competent to establish the proposition in the relevant domain and jurisdiction.

A digest only establishes the identity/integrity of referenced bytes. It does not establish entailment, truth, source credibility, jurisdictional applicability or assessor authority.

Need Intake records the claim and provenance. Claim Support evaluates the semantic relationship. Work Evidence, domain bodies, source owners and legal/safety authorities determine their own authenticity and competence requirements. Local ratification remains a separate decision.

## 2. Exact-bound evidence packet

Each claim support packet contains:
- claim identifier, exact claim digest and proposal digest;
- required semantic dimensions to inspect;
- source identifier and exact source digest;
- exact excerpt/observation digest and locator digest;
- source revision bound in both the source reference and assessment;
- support relation;
- per-dimension qualifier comparison;
- assessor kind, identity, method/version, model digest where machine-produced;
- assessment digest and assessment validity;
- independent review records binding to the exact assessment digest;
- qualified reviewer role, local authority reference, decision, validity and withdrawal.

The reference core does not fetch a source, parse its content, calculate canonical digests, verify signatures or decide whether the issuer/assessor is legally competent. Those are host and authority responsibilities.

## 3. Relation types and conservative semantics

A source/claim pair is classified as one of:

- **DirectSupport:** the selected source excerpt supports the claim as written.
- **PartialSupport:** only a subset or weaker form of the claim is supported.
- **Contradiction:** the excerpt conflicts with the claim.
- **ContextRequired:** additional source context or interpretation is required.
- **NotAssessable:** the available material cannot support a reliable decision.

PartialSupport is never upgraded to DirectSupport by prose, a model score or another positive citation. Contradiction is surfaced; it cannot be hidden by selecting a separate supporting source. High-consequence ambiguity routes to review or abstention.

## 4. Semantic qualifier dimensions

The reviewer or assessment method checks the dimensions required by the claim:

- Negation and polarity;
- Quantity, thresholds and ranges;
- Units;
- Temporal scope, date and freshness;
- Population and sample;
- System boundary and scope;
- Jurisdiction;
- Uncertainty and confidence;
- Causal strength (association, mechanism and causation remain distinct);
- Preconditions, conditions and exceptions.

For each dimension the assessment records whether it is Preserved, ClaimNarrowerThanSource, ClaimStrongerThanSource, Omitted, Contradicted, NotAssessed or NotApplicable. A qualifier omission or contradiction cannot produce Supported.

Examples:
- “not potable” cannot support “potable”;
- 10 L/min cannot support 10 L/day;
- a sample from one neighborhood cannot silently become a municipality-wide claim;
- “may reduce” cannot be strengthened to “will reduce”;
- an association cannot be stated as a causal effect without the necessary design and evidence.

## 5. Independent reviewer roles

Every claim names the reviewer roles required for a positive support disposition. Current roles include:

- **DomainExpert:** competent for the technical proposition.
- **SourceAuthority:** authorized to interpret or attest to the relevant source.
- **SafetyAuthority:** qualified to review safety-relevant claims.

The host must verify reviewer identity, role qualification, delegation, jurisdiction, revocation and freshness. The reference core checks that the review is independent from the assessor and bound to the exact assessment digest; it cannot authenticate those claims.

One current confirmation per required role is needed for each assessment. Different roles may be provided by different independent reviewers; duplicate decisions for the same assessment/role are rejected. RequestContext, RejectAssessment, expired, withdrawn or missing reviews do not qualify. A confirmed contradiction produces BlockedByContradiction. A machine assessment cannot self-certify, even when its model/provider is pinned.

## 6. Model and correlated-error handling

A machine assessment binds:
- model/provider identity and immutable model digest;
- prompt/policy/method identifier and version;
- source and excerpt digests;
- assessment digest;
- evaluation time.

Model agreement is not independent evidence when models share the same provider, training data, retrieval index, source error or reasoning family. The benchmark must record this correlation and test source-level common-cause failures. Independent human/domain review remains required for positive support.

## 7. Invalidation and temporal semantics

A support assessment is usable only while:
- the claim and proposal digests remain unchanged;
- the source/excerpt/locator digests still resolve to the same source revision;
- the assessment remains within its validity interval;
- every required reviewer decision remains current and unwithdrawn.

A source correction, new source revision, changed claim or expired review requires re-evaluation. The host must perform an atomic or reconciliation-safe freshness check immediately before any downstream system consumes the result.

## 8. Disposition

- **Supported:** all current assessments are DirectSupport; all required dimensions are qualifier-safe; every assessment has current independent confirmations for all required roles.
- **BlockedByContradiction:** at least one current independent review confirms contradiction.
- **HumanReviewRequired:** incomplete, partial, stale, disputed, role-incomplete, qualifier-unsafe or otherwise ambiguous support.
- **InsufficientEvidence:** no assessable source assessment is available.

No disposition authorizes a project, payment, resource reservation, professional licence, construction, medical action, safety operation or legal conclusion.

## 9. Standards and interoperability

- Use W3C PROV-DM to express source entities, assessment activities, assessor agents, derivation and versioned provenance bundles.
- Use W3C VC Data Model 2.0 and Bitstring Status List 1.0 for compatible credential/status inputs; do not invent a new credential or revocation format.
- Map requirement/claim quality to ISO/IEC/IEEE 29148:2018 and INCOSE requirement-writing guidance. Neither standard makes a source entail a claim.
- Use NIST AI RMF and ISO/IEC 42001 organizational governance as external risk-management frameworks; this crate is not certification.

## 10. Relation to conclusion-relative closure

Atomic claim support is necessary but not sufficient for a high-consequence decision. A cited excerpt may support an individual proposition while the overall conclusion still lacks material elements such as implementation evidence, authority, timing, exception handling or documentation.

Gupta (2026), “Regulatory evidentiary closure as a formal framework for citation faithfulness in large language models,” *Discover Artificial Intelligence* 6, article 1919, published 20 November 2026, formalizes this as conclusion-relative coverage over a domain-reviewed material-element mapping. The paper is theoretical and identifies inter-rater mapping reliability and field validation as open empirical needs.

The next layer is tracked separately in #6756 / PR #6757. Claim Support V1 intentionally stops at atomic claim/source support; it must never imply that a compliance, clinical, safety or other overall conclusion is closed.

Reference: https://doi.org/10.1007/s44163-026-02305-w

## 11. Non-goals

This core is not:
- an LLM-as-judge score or universal fact checker;
- a proof that an underlying statement is true;
- a source credibility registry;
- a licensing or statutory competence authority;
- a substitute for independent technical/safety review;
- an execution or authorization system.

The reference evaluator is a deterministic policy gate over host-supplied assessment and review snapshots. Semantic extraction and adjudication happen outside the pure Rust core and are separately benchmarked.
