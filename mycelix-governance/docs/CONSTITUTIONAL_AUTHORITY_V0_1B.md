# Mycelix Constitutional Authority v0.1B

Status: **experimental / stacked on MYC-CONST-001**

Tracking: #1198

This tranche hardens the constitutional authority vocabulary introduced by MYC-CONST-001. It does not activate runtime enforcement. Its purpose is to prevent four semantic failures before branch-crossing code is allowed to depend on the model:

1. collapsing sovereign decision powers and shared due-process/oversight entitlements into one authority category;
2. permitting delegated constitutional authority to become broader than its parent;
3. accepting delegated authority without validating the parent relationship; and
4. binding authority only to an abstract branch/guardian class rather than to a concrete holder.

## 1. Two authority categories

### Sovereign / constituted powers

A `ConstitutionalPower` is an authority to make or perform a constitutionally allocated public act: enact law, execute an appropriation, adjudicate a dispute, audit public expenditure, certify a mandate, or ratify constitutional change.

The separation theorem applies to these powers: every enumerated sovereign power has one constitutional owner class. Where multiple institutions must concur, the constitution should model separate powers or approvals in a protocol rather than silently making one power jointly owned.

### Shared constitutional entitlements

A `ConstitutionalEntitlement` is a procedural protection or access right needed for due process, oversight, or contestability. Examples include lawful record requests, evidence access/submission, notice, reasons, challenge, judicial review, protected oversight publication, and protected-disclosure intake.

An entitlement may be granted to multiple independent principals. Possessing an entitlement does **not** imply possession of a sovereign power.

This prevents reasoning such as:

- `may inspect evidence -> may decide guilt`;
- `may request records -> owns the audit function`;
- `may challenge an action -> may rewrite the governing policy`;
- `may receive a protected disclosure -> may prosecute the matter`.

Each entitlement grant carries an explicit holder, jurisdiction, lawful source, declared purpose, scope, information sensitivity, validity interval, and review path.

## 2. Authority class is not authority holder

`AuthorityPrincipal` identifies the constitutional **class** of an actor: constituent sovereignty, a branch, a guardian, or an automated-agent class.

Issued `ConstitutionalCapability` and `ConstitutionalEntitlementGrant` records separately require a non-empty `holder_id` identifying the concrete office, institution, commission, or constituent process that actually holds the authority.

This distinction prevents a dangerous ambiguity:

- `Integrity owns AuditAuthorityUse` is a constitutional allocation rule;
- `integrity:region-a holds capability cap-123` is an issued authorization fact.

Runtime signatures, revocation, conflict checks, appointment validity, audit receipts, and action-bound authorization must bind the concrete holder identifier rather than treating every actor in the same constitutional class as interchangeable.

The pure crate validates that a holder identifier exists. It does **not** yet verify the identifier against live identity/office state; that belongs to runtime integration.

## 3. Constitutional entitlement is not unlimited access

The split is not a surveillance bypass. A record-access entitlement is only a typed statement that access *may* be constitutionally available under the grant. Runtime enforcement must still bind the concrete request to its lawful purpose, scope, target/resource, minimization rules, privacy/secrecy constraints, and audit trail.

The inspected institution should not be the final judge of whether an independent constitutional overseer may inspect it, but neither should an overseer receive unrestricted access merely by naming itself independent.

Entitlement delegation is intentionally unsupported in v0.1B. A grant whose source is `Delegation` fails closed until entitlement-specific attenuation semantics exist.

## 4. Delegation is attenuation

A delegated `ConstitutionalCapability` must be a strict attenuation of its parent.

`validate_delegation` requires:

- both parent and child to be structurally and constitutionally valid;
- an active, non-revoked, non-expired parent;
- a parent that explicitly permits delegation and has remaining delegation depth;
- a distinct child capability ID;
- an exact parent-capability reference in the child source;
- identical constitutional power;
- equal or narrower jurisdiction according to a trusted jurisdiction resolver;
- a child validity interval contained within the parent's interval;
- strictly lower remaining delegation depth; and
- a child principal class that is itself constitutionally allowed to hold the power.

A child may therefore reduce authority but may not transform it.

Ordinary `ConstitutionalCapability::validate()` rejects any capability whose source is `Delegation`. A delegated capability is accepted only through parent-aware `validate_delegation()`. This is deliberate fail-closed behavior: a runtime caller cannot accidentally skip the parent relationship check and still obtain a successful constitutional validation.

If a parent is itself delegated, its parent-child edge must also be validated. Runtime chain traversal must validate from an authenticated root through every descendant and must reject duplicate/cyclic capability identities. Full persisted chain traversal and descendant revocation remain MYC-CONST-003 responsibilities.

## 5. Jurisdiction relation is externally resolved

The pure constitutional crate intentionally does not infer hierarchy from jurisdiction strings. `JurisdictionRelation` is the result of a trusted jurisdiction graph/profile resolver:

- `Same`;
- `ChildWithinParent`; or
- `BroaderOrUnrelated`.

Runtime code must compute this relation from authenticated jurisdiction state. It must never trust a delegating caller to self-assert that a broader jurisdiction is a child.

## 6. Intrinsically non-delegable powers

The initial model treats these as non-delegable constitutional authorities:

- emergency authorization;
- provisional emergency declaration;
- final constitutional review/remedy authority;
- public mandate certification;
- calling a constitutional convention;
- structural-constitution ratification;
- foundational-covenant ratification; and
- withdrawal of constituent delegation.

This does not mean staff or software cannot assist with implementation. It means the constitutional source of the decision cannot be transferred as a delegable sovereign capability.

The list is intentionally reviewable before runtime activation.

## 7. Provenance is required but not yet proven

`CapabilitySource` rejects empty source identifiers for charter, constituent-ratification, statute, judicial-order, emergency-protocol, and delegation sources.

This is only shape validation. v0.1B does not claim that the referenced source exists, is signed correctly, remains current, or lawfully authorizes the concrete action. MYC-CONST-003 must resolve and authenticate those facts before runtime use.

## 8. Automated agents

Automated agents continue to hold no enumerated sovereign constitutional power. v0.1B also prevents an automated agent from directly holding a constitutional entitlement grant.

This does not prohibit AI or software from executing bounded operational tasks on behalf of a lawful human/institutional principal. Those operational capabilities belong below the constitutional authority layer and must be separately scoped, auditable, revocable, and action-bound.

## 9. Failure-closed properties

The test suite now covers at least:

- delegated capability presented to root validation;
- power substitution during delegation;
- jurisdiction expansion;
- child activation before parent authority;
- child lifetime beyond parent authority;
- delegation-depth reset/amplification;
- incorrect parent reference;
- child reuse of the parent's capability ID;
- expired/revoked parent;
- non-delegable parent;
- delegation into a forbidden principal class;
- intrinsically non-delegable authority marked delegable;
- shared record access without adjudication authority;
- evidence access without prosecution authority;
- scoped entitlement purpose/review requirements;
- malformed provenance;
- unsupported entitlement delegation;
- missing concrete holder identity; and
- direct constitutional entitlement assignment to an automated agent.

## 10. Runtime boundary

This tranche remains semantic only. It does not yet:

- alter live voting;
- modify the governance DNA manifest;
- authorize actual protected-record disclosure;
- authenticate `holder_id` against live identity/office state;
- evaluate jurisdiction hierarchy from live state;
- traverse persisted delegation chains or propagate revocation to descendants;
- prove source existence/signatures/currentness;
- sign action-bound authorization envelopes; or
- enforce branch crossing at zome/DNA boundaries.

Those belong to MYC-CONST-003 after the semantic waist is qualified.

## 11. Core principle

> **Decision power should be scarce and exclusive; due-process and oversight entitlements should be explicit and composable; issued authority should identify its actual holder; delegated authority should only shrink.**
