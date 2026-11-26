# Craft Project Formation V1

- **Work item:** CRAFT-PROJECT-CORE-001 (#6729)
- **Status:** Reference-core implementation proposal; qualification pending
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## 1. Purpose

This contract turns a community need into a versioned project envelope with measurable outcomes, work packages and contextual capability requirements. It is deliberately not a job-posting, credential registry, project-management suite, resource allocator or payment service.

The key design change is that work packages declare the capability needed to achieve an outcome. People and organizations may then make consent-bound offers to cover those requirements, or a developmental offer can describe a bounded learning path.

## 2. Core records

| Record | Meaning | Owner boundary |
| --- | --- | --- |
| NeedToProjectV1 | Need, community, jurisdiction, project terms, authority reference, outcomes and work packages | Craft coordinates; host authenticates the project authority |
| ProjectOutcomeV1 | Measurable outcome and acceptance-policy reference | Outcome owner defines the measure and acceptance |
| CapabilityRequirementV1 | Capability profile, context, jurisdiction, policy, minimum distinct contributors and requirement kind | Project declares needs; independent authorities own assessment rules |
| CapabilityEvidenceRefV1 | Contextual evidence/verification receipt and current disposition | Work Evidence / qualified host owns verification and status |
| ParticipantConsentV1 | Participant acceptance or withdrawal for exact project terms | Participant controls consent |
| SupervisedDevelopmentRefV1 | Learning plan, learner agreement, mentor and external mentor-authority reference | Praxis / learning owner controls the learning record and mentor appointment |
| TeamProposalV1 | Requirement-level offers made by consenting participants | Craft evaluates coverage; no work authorization follows |

Taxonomies are referenced by vocabulary, concept, revision and digest. ESCO, OFO, a guild-local taxonomy or a future vocabulary can be used without becoming a universal Craft ontology.

## 3. Requirement classifications

- **SafetyCritical:** qualified, current, exact-scope evidence is mandatory. Supervision never counts as qualification.
- **Required:** coverage must meet the declared distinct-contributor threshold before the package can be considered covered.
- **Developmental:** a consented learning/supervision path may be reported as available, but it is not qualified coverage.
- **Preferred:** desirable capability, not an automatic exclusion gate.

The evaluator reports one disposition per requirement: Covered, BlockedBySafetyGate, HumanReviewRequired, DevelopmentPathAvailable, UnknownEvidence or Uncovered.

There is no aggregate person score or universal team score. Unknown is not a negative competency judgement; it simply cannot establish coverage.

## 4. Evidence matching

A qualification reference matches only when all the following are equal to the requirement:

1. capability vocabulary, concept, revision and definition digest;
2. work-context digest;
3. jurisdiction;
4. assessment-policy identifier, revision and digest;
5. subject identity, bound to the proposed contributor;
6. valid-from and valid-until interval at the evaluation time;
7. host-supplied disposition is Qualified.

Context mismatch, stale status, revocation, disputes and missing evidence never become qualified coverage. Disputed evidence requires human review. A different person's evidence is rejected as a structural error.

The crate validates a trusted snapshot. It does not authenticate signatures, issuer authority, statutory licences, adopted charters or revocation registries. These must be verified by the host and owning systems before evaluation.

## 5. Consent and learning

Every proposed participant accepts the exact project terms digest. A changed digest requires fresh consent. Withdrawal is effective at its declared time; a withdrawn participant cannot remain in the evaluated team.

A developmental assignment references a distinct consenting mentor, a host-verified mentor authority reference, an exact learning-plan digest and learner-agreement digest. These references establish only that a developmental pathway has been proposed. Praxis or the learning owner must validate and manage the real agreement.

A developmental pathway cannot satisfy a safety-critical requirement or authorize hazardous activity.

## 6. Non-goals and authority boundaries

The reference core does not:

- authenticate Identity DIDs, signatures, issuers or grantors;
- decide whether an institution's charter is valid;
- issue or verify professional credentials;
- infer statutory licensing from a private assessment;
- create or amend Holochain records;
- reserve tools, facilities, equipment or materials;
- create Manufacturing work orders;
- authorize a person to start work;
- accept physical outcomes or certify safety;
- create economic obligations, issue instruments, write journals or settle payments;
- rank people, guilds, communities or economic systems.

A Covered result is a scoped requirement-level classification, not permission to execute. The relevant project, safety, resource and work owners retain those decisions.

## 7. Composition

- Identity authenticates participants and issuer/controller relationships.
- Guild Governance evaluates locally adopted, scoped project authority.
- Professional Assessment Authority qualifies who may assess which capability.
- Work Evidence verifies evidence attestations and exposes current verification results.
- Praxis owns learner-controlled learning plans and evidence.
- Manufacturing owns work orders, material use and production events.
- Symtropy may qualify physical observations under its own assurance contracts.
- Finance and instrument issuers own all monetary and economic effects.
- Resource owners own inventory truth, allocation, custody, reservation and release.

No system should infer another system's facts from a link's mere existence.

## 8. Compatibility and versioning

Version 1 uses opaque, pinned references rather than hard-coding an occupational taxonomy or credential format. External mappings should preserve the source vocabulary identifier, revision, digest and any lossy transformation note.

A future serialization profile should be independently specified and tested against cross-language golden vectors before the Rust structs are treated as a wire format. This crate is a policy reference, not a canonical JSON or Holochain schema.

## 9. Qualification gate

Before production integration:

- compile and run all unit tests on the exact commit;
- run Clippy with warnings denied and Rust formatting checks;
- add property-based and mutation tests for duplicate IDs, consent changes, scope mismatches, expiry and revocation;
- independently implement the evaluator and reproduce frozen vectors;
- test withdrawal and revocation races at the host transaction boundary;
- verify host authentication of every authority/evidence/consent snapshot;
- review safety classifications with domain-specific practitioners;
- only then add Holochain and Praxis integration.

Authored tests are not passing tests. Reference-core tests are not host-state replay qualification. Host-state qualification is not field evidence.
