# GMD Offline Incident Envelope — Mycelix Protocol Note v0.1

**Status:** protocol proposal and schema fixture; not implemented, audited or deployed.  
**Date:** 2026-11-10  
**Parent:** [GMD-OFFLINE-COORD-001 #6018](https://github.com/Luminous-Dynamics/mycelix/issues/6018)  
**Foundation:** [SOV-DPI-005 #1030](https://github.com/Luminous-Dynamics/mycelix/issues/1030)

## 1. Purpose

Define a minimal, versioned message envelope for exchanging space-weather and essential-service observations across regions that may lose internet, cellular, satellite backhaul, GNSS time or central services.

This is a **store-carry-forward evidence protocol**, not a grid control plane. It carries observations, requests, proposals, acknowledgements and corrections. It never carries executable switching, dispatch, load-shed, relay-setting or protection commands.

The design principle is:

> Authentic origin is not semantic truth; integrity is not authority; delivery is not execution; a proposed plan is not an authorised action.

## 2. Four facts the envelope must never collapse

1. **Origin and integrity:** which key signed these exact canonical bytes?
2. **Authority claim:** which principal, credential, delegation, purpose, geography and consequence scope does the sender claim?
3. **Evidence semantics:** is the payload measured, published-derived, inferred, simulated or unknown, and which source/model/revision supports it?
4. **Local reliance:** what does this receiving node know about freshness, revocation, clock quality, conflicts, rights and permitted use at this time?

A valid signature proves only that the signed bytes verify under a key. It does not prove a sensor is calibrated, a measurement is true, an issuer has current authority, a model is valid in this region, or a service has been restored.

## 3. Envelope structure

The strict JSON Schema is `schemas/gmd_incident_envelope_v0_1.schema.json`. A synthetic format example is in `examples/gmd_incident_envelope_v0_1.example.json`.

The v0.1 envelope has three layers:

- **Protocol header:** protocol name and schema version.
- **Signed content:** message/event IDs, sender identity claim, claimed authority, issue/expiry times, causal parents, geography/subjects, evidence, correction links and an explicit authority ceiling.
- **Integrity block:** canonicalization profile, content digest, signature algorithm, signing-key reference and signature bytes.

The signature covers the canonicalized `content` object only; it does not cover receiver-local metadata. The verifier must reject duplicate JSON member names before canonicalization and must use the declared canonicalization profile exactly. JSON Schema validation is not a cryptographic verification.

## 4. Authority is bounded, not minted by the message

`authority_ceiling` is a hard maximum for the message's purpose:
- `OBSERVATION_ONLY`;
- `REQUEST_ONLY`;
- `PROPOSAL_ONLY`;
- `OWNER_ACKNOWLEDGEMENT_ONLY`.

There is deliberately no `ACTUATE`, `SWITCH`, `DISPATCH`, `SHED_LOAD`, `CHANGE_PROTECTION` or public-alert command type.

The embedded `authority_claim` is a claim to be checked locally against authenticated identity, delegation, role, purpose, geography, consequence limits, expiry and revocation generation. A receiver may attenuate that claim; it may never increase authority because the node is disconnected or its revocation view is stale.

The sender's `created_at_utc` and `expires_at_utc` do not establish freshness on their own. Each receiver records a separate, local receipt time and clock-quality statement. Local receipt metadata is not inserted into or re-signed as sender content.

## 5. Evidence semantics

Every evidence item states:
- evidence class: `MEASURED`, `PUBLISHED_DERIVED`, `INFERRED`, `SIMULATED` or `UNKNOWN`;
- source owner/reference, source revision/digest and rights status;
- observation interval, time-quality and uncertainty;
- subject and spatial coverage;
- measured quantity/value/unit or declared unknown state;
- model/version/domain only when modelled or inferred.

Do not upgrade a model estimate to a measurement because it is signed. Do not interpret missing coverage, no observation, stale evidence or unknown rights as a low-risk observation.

Rights are separate from authenticity. `UNKNOWN`, `RESEARCH_ONLY`, `COMMERCIAL_CONTRACT_REQUIRED`, `PARTNER_RESTRICTED`, `PUBLIC_REUSE_PERMITTED` and `REVOKED` are distinct dispositions.

## 6. Causal delivery and reconciliation

- Exact duplicate `message_id` + same content digest: idempotent duplicate.
- Same `message_id` + different content digest: equivocation/conflict; preserve both receipts and raise a conflict. Never select by timestamp, arrival order or lexical hash.
- A correction names the exact superseded `message_id` and content digest plus a reason. It appends a correction; it does not erase the previous record.
- `causal_context.parents` records the exact parent message IDs and digests known to the sender. Missing parents remain missing; do not invent order.
- During partitions, nodes may store and forward signed observations, requests and proposals within local resource budgets. They cannot create global authority or silently overwrite another branch.
- After reconnection, causal reconciliation is deterministic. Concurrent authority-bearing histories remain visible as `CONFLICTED` or `INDETERMINATE` until the required owner/governance process resolves them.

A valid signature on a false telemetry statement remains a validly signed false statement. Semantic cross-checking, independent sensors, model validation and operator review remain separate.

## 7. Time and freshness rules

Use UTC RFC 3339 timestamps in sender content. Record source clock quality and non-negative uncertainty. Receiver-local receipt metadata includes its own clock uncertainty.

If the source clock, receipt clock, expiry or revocation generation is unknown:
- mark freshness as unknown or stale;
- do not increase the authority ceiling;
- do not use time ordering to resolve a conflicting claim;
- require explicit reconciliation before any consequential reliance.

The implementation must define monotonic local sequence/boot-generation handling for nodes whose wall clock is unreliable. A source sequence is not a substitute for verified wall time.

## 8. Required reference harness scenarios

Before any Holochain/Civic integration, implement an effects-disabled deterministic reference harness and test:

| Scenario | Required disposition |
|---|---|
| Same message delivered repeatedly | Idempotent duplicate |
| Same message ID, changed payload digest | Conflict / equivocation |
| Parent arrives after child | Preserve causal gap; reconcile when parent arrives |
| Two corrections supersede same predecessor | Preserve both; no last-writer-wins |
| Expired credential / stale revocation epoch | Authority attenuated or insufficient |
| Clock unknown / offset / drift | Freshness unknown; no authority increase |
| Authentic but false telemetry | Signature validates; semantic truth remains unverified |
| Restricted source rights | No reuse/disclosure outside permitted purpose |
| 72-hour partition and restart | Durable local evidence; bounded autonomy; deterministic later reconciliation |
| Unknown outcome of an external effect | Indeterminate; no blind retry |

Metrics: durable message survival, duplicate suppression, causal-order correctness, unauthorized disclosure, authority escalation, equivocation visibility, reconciliation convergence, priority delivery latency under bandwidth/power constraints, bytes/CPU/storage per event, and false trust acceptance for authentic-but-false content.

P0 targets are zero unauthorized disclosure and zero authority escalation across the frozen test corpus. These are test-panel results, not proof of security under all attacks.

## 9. How this composes with the four-system architecture

- **Sol-Atlas:** supplies spatial subject references, dependency coverage and read-only impact evidence.
- **Symtropy:** supplies versioned physical-model receipts (E-field, induced voltage, GIC, transformer response) with units and domain/uncertainty.
- **Symthaea:** supplies risk assessments and candidate plans as explicitly inferred/proposed evidence, never as observed truth or authority.
- **Mycelix:** authenticates, scopes, stores, forwards, deduplicates and reconciles envelopes under local autonomy budgets.

Qualified utility operators, local authorities and service owners remain responsible for emergency decisions. Mycelix provides accountable coordination, not a universal decision-maker.

## 10. Immediate next implementation slice

1. Strict JSON parser that rejects duplicate object keys and unknown fields.
2. Schema validation plus canonicalization and signature-verification adapter (signing key comes from the authenticated Mycelix identity layer; do not invent a second identity registry).
3. Content digest / idempotency key and deterministic receipt.
4. Correction and parent-causality evaluator with explicit `DUPLICATE`, `CONFLICTED`, `PENDING_PARENTS`, `ACCEPTED_AS_EVIDENCE`, `INSUFFICIENT_AUTHORITY` and `STALE` dispositions.
5. Fault-injection replay corpus before multi-conductor/Holochain integration.

The schema example deliberately uses a dummy signature byte string. It is a format fixture only and must fail real cryptographic verification. No private key, real identity, live data, asset topology or actionable grid command is included.

## 11. Claim ceiling

This document and schema are a protocol proposal. They do not prove cryptographic implementation, Holochain DHT behavior, partition safety, identity freshness, semantic truth, operational availability or emergency readiness. The Mycelix Civic cluster remains pre-alpha; deployment requires multi-conductor tests, independent security review, partner/governance agreement and owner-specific operational exercises.
