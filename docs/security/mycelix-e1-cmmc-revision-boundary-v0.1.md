# Mycelix E1 CUI/CMMC Revision Boundary v0.1

**Status:** executable reference mapping  
**Claim ceiling:** `ReferenceModelOnly`  
**Snapshot:** 2026-10-04

This artifact prevents the E1 engineering program from accidentally equating the current NIST SP 800-171 Rev. 3 engineering target with the operative CMMC basis.

## Current boundary

As of October 4, 2026, the current DoW CIO public CMMC page states that CMMC Phase II was suspended on July 13, 2026. It says Phase I self-assessment requirements remain in place and, during the pause, Level 2 uses the 110 requirements from NIST SP 800-171 Rev. 2. citeturn160550search1

Separately, DFARS 252.204-7021 remains a contractual surface whose required CMMC level is inserted for the particular solicitation/contract. Its current November 2025 text defines multiple CMMC statuses, including Level 2 self/C3PAO paths, and requires a current status where the clause applies. citeturn160550search0turn160550search2

Therefore the repository maintains two explicit tracks:

```
E1 engineering architecture -> SP 800-171 Rev.3 family/requirement target
CMMC operational/contract surface -> current 32 CFR/DFARS basis and solicitation-specific level
```

They are related, but they are **not automatically equivalent**.

## Why this matters

A semantic PASS such as:

```
RATS verifier: PASS
PEP decision: PASS
release benchmark: PASS
```

does not create CMMC status.

Likewise:

```
SP 800-171 Rev.3 mapping
!=
CMMC Level 2 status
```

The distinction must remain visible to engineers, assessors, contracting personnel, and future compliance tooling.

## Mapping discipline

The machine-readable matrix uses:

```
AlreadyQualified
ImplementedUnqualified
Designed
CandidateMapping
Gap
OrganizationDecisionRequired
ContractDecisionRequired
IndependentAssessorRequired
NotApplicable
```

A mapping is evidence about correspondence between a feature and a requirement area. It is not certification evidence unless the applicable assessment authority accepts it.

## Current E1 posture

The engineering layers now provide candidate technical evidence for:

- access control;
- identification/authentication;
- audit/evidence;
- configuration/provenance;
- security assessment;
- system/communications protection;
- system/information integrity.

The matrix deliberately leaves organizational, personnel, physical, maintenance, media, and incident-response dependencies visible rather than inventing software equivalents.

## Qualification vectors

Twelve structural vectors ensure the boundary itself cannot drift:

- Rev.3 is not silently declared the operative CMMC basis;
- semantic mappings are not converted into CMMC status;
- contract-specific CMMC level remains an external decision;
- missing organizational/physical evidence remains externally owned;
- independent assessment remains external;
- Phase II suspension is not ignored;
- green code tests do not become CMMC status;
- the Rev.3 engineering target remains explicit.

## Claim ceiling

This artifact establishes only revision/ownership bookkeeping and engineering mapping semantics.

It does not establish CMMC status, legal compliance, contract applicability, NIST compliance, assessment readiness, or government authorization.
