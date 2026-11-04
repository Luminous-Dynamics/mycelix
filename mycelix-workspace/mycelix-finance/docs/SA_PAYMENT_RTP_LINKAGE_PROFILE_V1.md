# RTP-to-Payment Message Linkage Profile v1

**Status:** synthetic interoperability and evidence contract. No live PayShap mapping, payment connector or real-money effect.

## Why this exists

Request-to-Pay is an initiation/messaging layer. The payer's response and the actual credit transfer are separate messages. ISO 20022's RTP best-practice note explicitly identifies the need to link a later `pacs.008` transfer to the original `pain.013` request so the requester and their bank know which payment fulfilled it.

The corpus maps:
- `pain.013.001.12` — Creditor Payment Activation Request;
- `pain.014.001.12` — Creditor Payment Activation Request Status Report;
- `pacs.008` — FI-to-FI Customer Credit Transfer;
- `pacs.002` — FI-to-FI Payment Status Report.

The exact active message versions and bilateral usage rules must be obtained for the actual participant/scheme. A generic ISO message identifier does not imply PayShap uses that exact version or mapping.

## Four-corner model

Represent the Payee, Payee RTP Service Provider, Payer RTP Service Provider and Payer as distinct parties. Represent the debtor/creditor agents and clearing/settlement operators on the subsequent transfer leg. EPC SRTP v4 implementation guidance defines separate payee-to-provider, inter-provider and payer-to-provider message spaces. EPC's current trust/security model v1.1 uses authentication, message integrity and JWS sealing for SRTP API communications.

A valid seal proves message origin/integrity under the selected key/profile. It does not prove the invoice is legitimate, the payer saw the content, the payer consented, the payment executed or the beneficiary was credited.

## Typed stages

`pain.013 request → pain.014 response → payer payment instruction → pacs.008 transfer → pacs.002 instruction status → recipient account credit → settlement finality`

The order and evidence prerequisites are profile-specific. Each transition has its own identity, participant role, authentication evidence, source message reference, event time and claim ceiling.

**Non-equivalences:**
- request delivered != payer accepted;
- pain.014 accepted != payment instruction created;
- payment instruction != pacs.008 transfer sent;
- pacs.002 pending/accepted != beneficiary credited;
- beneficiary account credit != interbank settlement finality;
- matching amount/reference != request-to-transfer linkage.

## Linkage tuple

The synthetic v1 record binds request ID and digest, message ID/version, payer/payee participant IDs, payer instruction ID, end-to-end ID, pacs.008 message ID and transaction ID, amount/currency, status message references, participant role, and independent beneficiary-credit/finality receipt. No single identifier is assumed globally unique; the correlation namespace and matching tuple must be derived from each scheme's active usage guideline.

A duplicated pacs.008 with the same transfer fingerprint is idempotent. A second transfer with a different transaction ID for one request is ambiguous and quarantined. Two distinct requests linked to one transfer are likewise quarantined. A status without a matching pacs.008 is an orphan status, not payment proof.

## South African claim ceiling

The South African profile remains deliberately unqualified. SARB's PSMB governance page identifies the institutional transition, but does not provide the PayShap Request message-field mapping. Production qualification needs the current PayShap/PayInc participant message specifications, status and reason-code mappings, identifier namespaces, authentication model and authorised sample traces. Do not infer these from EPC or generic ISO market practice.

## Benchmarks

Hard gates: no transfer link based only on amount/reference; no instruction without exact payer acceptance; no status promotion to credit/finality; no stale/revoked key or wrong-role message acceptance; no duplicate economic effect; one-request/one-transfer and amount/currency reconciliation; deterministic replay of frozen inputs; preserve unmatched/ambiguous outcomes as explicit states.

Measure latency and coverage separately at request→response, response→instruction, instruction→pacs.008, pacs.008→status and transfer→recipient availability. Track false matches, unmatched messages, duplicate suppressions and ambiguous links. Do not optimise for link coverage by weakening evidence.

## Claim ceiling

This is synthetic evidence-linkage conformance only. It establishes no South African live mapping, real bank interoperability, funds movement, settlement finality, legal entitlement or current participant availability.
