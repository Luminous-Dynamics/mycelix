# AeroCommons Identity Profiles V1

## Status

Design contract for AEROCOMMONS-005.

This document freezes the semantic classes of identity before a Holochain graph is introduced. It does not introduce a new universal digest algorithm or replace Mycelix's existing evidence/source identity mechanisms.

## Purpose

AeroCommons crosses several identity domains that must never be collapsed:

1. artifact identity — the engineering thing or logical object;
2. content identity — an exact payload;
3. configuration identity — a particular engineering state;
4. source-evidence identity — an authored evidence observation/record;
5. execution identity — one performed operation, inspection, measurement, or test instance;
6. epistemic-relation identity — one authored assertion about a relationship.

The same bytes may occur in different identity domains, and one domain may refer to another. Equality in one domain MUST NOT imply equality in another.

## Identity matrix

| Kind | Identifies | Canonical preimage | Mutable? | Supersession |
| --- | --- | --- | --- | --- |
| Artifact | logical engineering object | domain-defined stable artifact descriptor | yes | explicit lineage |
| Content | exact payload/representation | canonical payload bytes + declared content profile | no | new content identity |
| Configuration | engineering state | canonical ordered member/parameter/requirement manifest | no | new configuration identity |
| SourceEvidence | authored evidence record | existing Mycelix source/evidence identity profile | append-only record | correction/supersession |
| Execution | one performed operation | execution manifest + bound inputs/toolchain/context | no | new execution |
| EpistemicRelation | one authored consequential relationship | canonical relation tuple + basis/scope/lifecycle profile | no | new relation with predecessor |

The exact cryptographic algorithms and canonical serialization profiles are inherited from or separately frozen by the owning subsystem. This table freezes the meaning of the identity, not an unapproved second hashing scheme.

## Profile requirements

Every identity reference MUST carry:

- kind;
- id;
- profile.

profile is mandatory because the identifier alone is not enough to interpret an external identity. A profile identifies the canonicalization and binding rules used by the producing subsystem.

A consumer MUST reject or quarantine a reference whose profile is unknown when the profile is required to verify a consequential relation.

## Equality rules

The following implications are forbidden:

- Content equality => Artifact equality.
- Artifact equality => Configuration equality.
- Configuration equality => Execution equality.
- SourceEvidence equality => Claim equality.
- Claim equality => EpistemicRelation equality.
- Holochain EntryHash equality => engineering identity equality.
- Holochain ActionHash equality => content identity equality.

Conversely, one artifact may have multiple content representations, one configuration may reference many artifacts, one claim may have multiple source-evidence records, and one relation may cite multiple independent evidence records.

## Holochain mapping

Holochain supplies protocol identities for authored records and content-addressed entries. Its ActionHash is an action instance identifier and its EntryHash identifies entry content; neither is a substitute for the domain identities above.

AeroCommons therefore stores engineering identity explicitly and may additionally retain Holochain hashes as transport/provenance bindings.

Validation MUST remain deterministic and structural. Holochain validation can reject malformed or unauthorized records and can resolve addressable dependencies, but it cannot establish that an engineering claim is physically true or that a configuration is airworthy.

## External payload binding

Large engineering payloads remain outside the Holochain entry envelope where appropriate.

A content identity SHOULD bind:

- canonical media type;
- canonicalization profile;
- exact payload bytes or an existing Mycelix content commitment;
- optional external URI/location metadata that is explicitly non-authoritative.

Location/URL equality MUST NOT be treated as content equality.

## Configuration identity

A configuration identity is a frozen engineering state, not merely a name or version string.

Its canonical manifest SHOULD bind:

- predecessor configuration, when applicable;
- ordered component/artifact identities;
- parameter/value identities;
- applicable requirements;
- declared external dependencies;
- configuration profile version.

A configuration fork creates a new configuration identity. It MUST NOT mutate the identity of its predecessor.

## Execution identity

Execution identity distinguishes repeated observations of the same artifact under different circumstances.

It SHOULD bind:

- operation kind;
- subject artifact/configuration;
- input identities;
- method/procedure identity;
- toolchain/instrument identity;
- operator/producer identity;
- execution context;
- declared time representation;
- resulting observation/evidence identity.

Two executions with identical inputs and outputs are still distinct executions unless an explicit equivalence profile says otherwise.

## Source-evidence identity

AeroCommons MUST reuse the established Mycelix evidence/source identity substrate rather than create a second universal evidence ledger.

Engineering adapters may wrap or reference source-evidence records, but the underlying evidence identity remains governed by the existing canonical profile.

An AeroCommons adapter MUST preserve:

- observation vs inference distinction;
- source identity;
- provenance;
- uncertainty;
- correction/supersession;
- contestability;
- controlled disclosure semantics.

## Epistemic-relation identity

A relation is not a navigation edge when it asserts something consequential.

Examples:

- execution A reproduces source evidence B;
- evidence C contradicts evidence D;
- analysis E depends on configuration F;
- test G demonstrates requirement H.

Such a relation receives its own identity and basis. A Holochain link MAY index it, but the link itself MUST NOT be the evidence-bearing assertion.

## Migration

Profiles are versioned.

A profile migration MUST produce an explicit mapping or new identity. It MUST NOT silently reinterpret an existing identifier under a different canonicalization profile.

When an external standard changes representation (for example a STEP/QIF serialization or schema revision), the adapter MUST state whether the change preserves content identity, creates a new representation identity, or changes the engineering artifact/configuration identity.

## Qualification corpus

An implementation claiming conformance to this profile MUST test at least:

1. identical payload, different execution;
2. identical payload, different artifact;
3. same artifact, forked configuration;
4. duplicate evidence bytes from different source events;
5. one claim supported by multiple source-evidence records;
6. one source-evidence record cited by multiple relations;
7. superseded relation with predecessor;
8. corrected/superseded evidence;
9. private payload with public commitment;
10. unavailable external artifact;
11. unknown profile;
12. profile migration;
13. Holochain EntryHash retained separately from domain identity;
14. Holochain ActionHash retained separately from execution identity;
15. configuration with reordered but semantically identical manifest;
16. configuration with one changed parameter.

## Non-goals

This profile does not:

- certify engineering correctness;
- establish material properties;
- establish airworthiness;
- define a global trust score;
- turn peer attestation into physical evidence;
- prescribe a blockchain;
- define a new universal content hash;
- make Symthaea an engineering authority.

## Core invariant

**Identity tells us what record or thing we are talking about. It does not tell us whether the claim about that thing is true.**
