# GCPN CAP 1.2 Provenance Sidecar Profile v0.1

**Status:** proposed interoperability profile; not a CAP standard, national profile, or operational alerting system.  
**Parent research:** GCPN issue #5593; executable synthetic harness PR #5591.  
**Safety scope:** public/synthetic warning records and federation evidence only. No operational alert dispatch, weapon-control, tactical military data, or live sensitive source details.

## 1. Design decision

GCPN MUST NOT create a competing public-warning message format. CAP 1.2 remains the alert interchange object. A GCPN sidecar carries provenance, evidence-state, federation and custody information that should not be forced into a CAP payload.

CAP is an open, non-proprietary, all-hazards warning-message format. CAP does not define a particular application or telecommunications method. A CAP alert has an alert-level segment and may contain multiple info segments; info may be repeated for different parameters or languages and may contain area/resource structures. CAP supports urgency, severity, certainty, geographic information, update/cancel relationships, and optional digital signatures. These semantics must survive any sidecar association or round-trip transformation.

Normative reference:
- OASIS Common Alerting Protocol Version 1.2: https://docs.oasis-open.org/emergency/cap/v1.2/CAP-v1.2.html
- ITU-T X.1303 bis: https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Documents/2020/T-REC-X.1303bis-201403-.pdf

The CAP XML namespace is `urn:oasis:names:tc:emergency:cap:1.2`. CAP 1.2 specifies an optional enveloped XML Signature. A CAP consumer must not reject a message solely because it cannot verify an included signature; it should inform the user about the validation failure. The GCPN profile must never reinterpret the mere presence of a signature as evidence that the physical observation is accurate.

## 2. Separation of concerns

| Concern | Authority / representation | GCPN handling |
|---|---|---|
| Public warning semantics | CAP 1.2 message fields and the applicable national CAP profile | Preserve without rewriting or lossy normalization |
| Sender authenticity | Signature/key validation under the sender's profile | Record validation result and verifier profile separately |
| Observation validity | Source-specific methodology and independent corroboration | Evidence state is explicit; authenticity does not imply accuracy |
| Public-warning authority | Competent national/local authority and applicable legal process | Carry a reference to an authority assertion; never infer authority from signature alone |
| Public delivery | Dissemination channels and receivers | Store receiver-side receipts separately from CAP content and server send counts |
| Federation provenance | GCPN versioned sidecar | Source lineage, custody, purpose constraints, dependency family and audit references |
| Physical truth / legal effect | Competent public authority or treaty institution | Never adjudicate from sidecar metadata alone |

## 3. Binding contract

The sidecar is versioned independently of CAP and binds to one exact CAP message representation.

Required binding fields:
- `profile_id`: `gcpn-cap-sidecar-v0.1`;
- `cap_namespace`: exact CAP namespace;
- `cap_identifier`, `cap_sender`, `cap_sent`: exact CAP alert identifiers;
- `payload_digest.algorithm`: approved digest algorithm identifier;
- `payload_digest.scope`: `exact-xml-octets` in v0.1;
- `payload_digest.value`: digest of the exact CAP XML octets associated with this sidecar;
- `sidecar_id`, `created_at`, `profile_version`.

**Why exact XML octets in v0.1:** XML canonicalization, namespace rewriting, whitespace and signature transforms are easy places to create incorrect assumptions. This profile does not invent a canonicalization algorithm. Two semantically similar but byte-different XML documents have different exact-octet digests and must be rebound explicitly. A future canonicalized semantic-digest profile requires a separately reviewed, test-vector-backed algorithm choice.

A sidecar with a mismatched identifier, sender, sent timestamp, namespace, digest algorithm, digest scope, or digest value is rejected as unbound. It must not be attached to a different CAP message by fuzzy matching or title/time similarity.

## 4. Sidecar object model

### 4.1 Evidence references

Each evidence reference SHOULD include:
- opaque evidence reference ID;
- issuing member namespace;
- source-family pseudonym (for dependence accounting, not a public identity);
- source observation time and record-receipt time;
- evidence status: `validated`, `unverified`, `contradictory`, `stale`, `missing`;
- integrity/authenticity result and verifier-profile reference;
- custody predecessor digest or explicit genesis marker;
- permitted purpose, audience and retention class;
- privacy classification and disclosure policy reference.

The sidecar SHOULD NOT contain raw sensitive sensor coordinates, private-person data, restricted facility information, or source details unnecessary for downstream verification. Public alerts retain the geography needed by the competent authority in the CAP payload; sensitive provenance detail stays in the appropriate restricted evidence system.

### 4.2 Authority assertion

An authority assertion is a reference to an external, governed credential/mandate:
- authority issuer/member;
- jurisdiction or delegated scope identifier;
- permitted action class (e.g. issue/update/cancel public alert);
- validity interval;
- authority credential reference and revocation-check result;
- applicable national/profile policy version.

The sidecar records the reference and validation state. It does not create the authority, expand jurisdiction, or override local law. A cryptographic signature from an individual or sensor is not equivalent to delegated authority to issue a public warning.

### 4.3 CAP lifecycle linkage

CAP `msgType`, `references`, `incidents`, `sent`, `status` and `scope` remain authoritative for CAP message lifecycle semantics. GCPN sidecar lifecycle entries may record provenance and custody for each CAP message, but must not replace CAP update/cancel chains or silently mutate the original object.

A new update/cancel message gets a new sidecar binding to its own CAP identifier, sender, sent timestamp and digest. Related messages are linked with CAP references as prescribed by the chosen CAP profile.

### 4.4 Receiver evidence

Receiver receipts are separate append-only records:
- receipt ID;
- CAP binding tuple and digest;
- receiving system pseudonym / cohort (privacy-preserving);
- channel class;
- receiver-side time;
- receipt type (`received`, `rendered`, `accessible-format-available`, `acknowledged`);
- accessibility mode and language as non-identifying aggregate categories;
- verifier/custody reference.

Do not infer human comprehension or action from a transport receipt. Those require ethically reviewed, voluntary evaluation and separately defined measures.

## 5. Synthetic conformance requirements

The v0.1 sidecar contract expects tests for:
1. exact-byte digest match;
2. CAP identifier/sender/sent mismatch;
3. wrong namespace or profile version;
4. stale sidecar and replay;
5. CAP update/cancel chain with distinct bindings;
6. multi-info/multi-language preservation;
7. unknown national-profile extension preservation or explicit unsupported-state report;
8. signature present but unverifiable: message remains parseable and validation failure is visible;
9. valid signature but unverified/contradictory observation;
10. authentic evidence with unauthorized issuer;
11. absent authority assertion;
12. receiver receipt not falsely inferred from server send report;
13. sensitive provenance not copied into public CAP payload;
14. expired message and old sidecar replay;
15. sidecar revocation / key rotation with historical custody retained.

Hard gates:
- zero CAP semantic field loss within the declared preservation set;
- zero acceptance of mismatched or replayed sidecars;
- 100% sidecar linkage to exact CAP message identity and exact XML octets;
- zero automatic promotion from signature-valid to measurement-validated;
- zero automatic promotion from evidence-authenticated to public-warning-authorized;
- zero inferred receiver receipts from server dispatch counts;
- unknown fields or profile-specific extensions are preserved byte-for-byte where supported, otherwise explicit incompatibility is returned.

## 6. Comparison protocol

Compare reference architectures only under identical synthetic inputs:
- CAP-only message exchange with the existing reference dissemination adapter (baseline);
- regional federated sidecar exchange with independent member custody;
- local-first federation with local evidence and fallback under cross-border partition.

Measure:
- CAP parse/validation conformance and round-trip semantic equality;
- sidecar binding and verification overhead;
- evidence-lineage completeness;
- local continuity during partition;
- p50/p95 binding and validation latency;
- receiver-side delivery receipts and accessibility coverage by voluntary receiver cohort;
- privacy leakage and metadata correlation;
- recovery and reconciliation after connectivity returns;
- operator burden and lifecycle cost.

Do not claim superiority over a named national warning service until a compatible public adapter and independent, like-for-like receiver evidence exist. CAP conformance alone is not proof of warning effectiveness.

## 7. Versioning and migration

- Sidecar version MUST be explicit and independent from CAP version.
- Unknown mandatory sidecar features fail closed for the sidecar operation while preserving the original CAP message.
- Unknown optional extensions are preserved or explicitly surfaced; do not silently discard.
- Profile upgrades require forward/backward compatibility tests and immutable fixture versions.
- Key revocation, source withdrawal and member exit must not erase prior evidence custody or historical disputes.
- A new profile version must not reinterpret old digest scope or authority assertions.

## 8. Non-claims

This profile does not validate any live alert network, make public warnings, certify the truth of observations, guarantee end-user receipt, set hazard-response policy, create supranational authority, or provide missile-defense capability. It is an interoperability research contract and synthetic benchmark target.
