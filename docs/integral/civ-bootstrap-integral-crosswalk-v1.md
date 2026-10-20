# Mycelix CIV / Integral Bootstrap Crosswalk v1

**Status:** Design and conformance artifact

## Scope

"CIV bootstrap" is ambiguous because Mycelix contains both network peer bootstrap and the Mycelix Civic (CIV/CivOS) hApp. This artifact focuses on CIV/CivOS while explicitly separating network discovery from civic authority.

Network bootstrap establishes connectivity/discovery. It must not silently establish civic identity, membership, governance authority, credential validity, or economic entitlement.

## Semantic comparison

| Concern | Integral COS | Mycelix Civic | Bootstrap rule |
|---|---|---|---|
| Identity | actor/resource identity bound to evidence | DID-centric Party | identity precedes role |
| Role | assignment/context | PartyRole | role is contextual, not universal authority |
| Evidence | source observation + provenance | Evidence + CustodyEvent + Verification | verification does not erase provenance |
| Temporal validity | freshness/validity windows | timestamps/statuses | current state does not erase history |
| Capability | distinct from availability | role/credential context | capability != authorization |
| Authorization | explicit decision/mandate boundary | civic action workflows | bootstrap cannot self-authorize |
| Execution | observed/effect evidence | CrossHappAction/status | requested != executed |
| Outcome | downstream observation | result/effect references | completion != real-world outcome |
| Federation | foreign recognition preserves origin | civic bridge/cross-hApp calls | bridge != provenance rewriting |
| Correction | append/supersession lineage | evidence/case history | corrections preserve history |

## Bootstrap semantic chain

```text
BootstrapIdentity
    -> IdentityEvidence
    -> RoleContext
    -> CapabilityEvidence
    -> ExplicitAuthority
    -> AuthorizedAction
    -> ExecutionReceipt
    -> OutcomeEvidence
```

These are distinct claims. In particular:

- identity != role;
- role != capability;
- capability != authorization;
- authorization != execution;
- execution != successful outcome;
- reputation != evidence of execution;
- network connectivity != civic membership;
- civic membership != governance authority.

## Network bootstrap firewall

The network bootstrap operator guide describes primary/secondary/tertiary servers, local cache, and DHT discovery. These are connectivity mechanisms.

A CIV conformance adapter should reject:

```text
bootstrap_server_response -> civic_authority
peer_discovery -> identity_verification
peer_registration -> governance_membership
network_reachability -> credential_validity
bootstrap_tier -> trust/rank
```

A discovered peer is a transport/contact observation unless separate civic evidence qualifies a stronger claim.

## Integral-to-CIV candidate mapping

```text
Integral actor context
        |
        v
CIV Party / DID
        |
        +--> PartyRole
        +--> Evidence / Verification
        |
        v
explicit CIV authorization
        |
        v
CIV CrossHappAction
        |
        v
execution/effect evidence
```

This is an interoperability proposal, not Integral ratification and not a claim that current CIV code implements every transition.

The source-ownership rule is:

> The system that observes or owns a fact remains the source owner; downstream systems may reference, assess, or project it without silently replacing it.

## Adversarial corpus

### Positive

| ID | Case | Preservation |
|---|---|---|
| CIV-BOOT-POS-001 | DID -> Party | identity |
| CIV-BOOT-POS-002 | Party -> contextual role | role context |
| CIV-BOOT-POS-003 | evidence -> qualified verification | provenance/evidence |
| CIV-BOOT-POS-004 | explicit authority -> civic action | authority |
| CIV-BOOT-POS-005 | civic action -> execution receipt | lifecycle |
| CIV-BOOT-POS-006 | foreign attestation -> recognized input | foreign origin |
| CIV-BOOT-POS-007 | corrected evidence -> appended lineage | history |
| CIV-BOOT-POS-008 | network discovery -> contact observation | transport only |

### Negative

| ID | Invalid promotion | Result |
|---|---|---|
| CIV-BOOT-NEG-001 | peer discovery -> civic identity | reject |
| CIV-BOOT-NEG-002 | DID possession -> universal civic authority | reject |
| CIV-BOOT-NEG-003 | PartyRole -> unrestricted authorization | reject |
| CIV-BOOT-NEG-004 | reputation -> credential validity | reject |
| CIV-BOOT-NEG-005 | credential -> capability without scope/validity | reject |
| CIV-BOOT-NEG-006 | capability -> authorization | reject |
| CIV-BOOT-NEG-007 | authorization -> execution | reject |
| CIV-BOOT-NEG-008 | execution status -> successful real-world outcome | reject |
| CIV-BOOT-NEG-009 | bridge recognition -> local-origin evidence | reject |
| CIV-BOOT-NEG-010 | correction -> historical rewrite | reject |
| CIV-BOOT-NEG-011 | bootstrap tier -> trust score | reject |
| CIV-BOOT-NEG-012 | network reachability -> governance membership | reject |

## Qualification ladder

```text
Conceptual
  -> Crosswalked
  -> Schema-Mapped
  -> Executable Conformance
  -> Runtime Adapter
  -> Federated Conformance
  -> Pilot Observation
  -> External Validation
```

A passing crosswalk or semantic corpus does not establish runtime compatibility, civic effectiveness, governance legitimacy, safety, legal validity, or Integral endorsement.

## Claim ceiling

This artifact does not claim Integral endorsement, that Mycelix Civic implements complete COS, legal identity, universal authority, or that network bootstrap is governance infrastructure.

## Next executable slice

Add a typed CIV bootstrap conformance module beside the existing Economic Fabric/COS modules:

```text
CivBootstrapIdentity
CivRoleContext
CivEvidence
CivAuthorization
CivAction
CivExecutionReceipt
CivOutcomeEvidence
```

It should preserve `source_schema`, `source_revision`, `object_id`, `domain`, `origin`, `source_event`, `predecessor`, `evidence_ids`, and correction lineage.

The goal is one evidence-preserving conformance substrate shared across Integral/COS, Economic Fabric, and CIV without collapsing them into one ontology.
