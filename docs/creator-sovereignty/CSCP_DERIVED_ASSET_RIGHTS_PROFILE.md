# CSCP Derived-Asset Rights Propagation Profile

**Status:** research/conformance proposal v0.1  
**Owner:** Mycelix Creator Sovereignty Control Plane (CSCP)  
**Related work:** #5181, #5186, #5190, #5166, #6003, #6006  
**Claim ceiling:** a policy decision is evidence of a system decision under a declared policy snapshot; it is not a legal adjudication or proof that the asserting party possessed the claimed authority.

## 1. Purpose

A cultural object can have many representations and derivatives: photographs, crops, scans, meshes, point clouds, OCR text, captions, thumbnails, embeddings, vector-index entries, exports, analytics and model-training examples. Restrictions attached only to the source record are insufficient. A derivative can leak the source's protected content even when the original file remains inaccessible.

This profile defines a CSCP-owned contract for operation-specific rights and restrictions across the full derivation graph. It extends the existing CSCP identity and scoped-grant infrastructure. It does not create a second rights ledger or a new source of legal authority.

The security objective is:

> No derivative, transformation, index, cache, export or model workflow may acquire broader permissions than the complete set of source resources and competent authorities permit.

This is a conservative technical invariant, not a claim that software can resolve every legal or cultural dispute.

## 2. Standards strategy

Reuse standards rather than inventing a parallel policy vocabulary:

- **W3C ODRL Information Model 2.2** provides the core concepts of assets, parties, permissions, prohibitions, duties, actions and constraints. CSCP should define a versioned sector profile for artwork-specific actions, resource scope, authority evidence and conservative conflict semantics.
- **W3C Verifiable Credentials 2.0** can carry issuer-bound claims and proof material. Credential verification establishes proof/integrity and status, not the truth of the claim or the issuer's legal/cultural competence.
- **W3C PROV-O** can describe derivation, agents, activities and lineage.
- **IIIF Authorization Flow API** is relevant to protected image/content delivery. It is not a complete policy engine for every CSCP derivative or AI-processing action.
- Existing CSCP approval, replay and event-lineage mechanisms remain authoritative for their current responsibilities.

A standards mapping must record the exact version, profile and mapping-loss report. Unknown fields must not be silently discarded during import/export.

## 3. Resource and subject scope

Every policy rule MUST bind to a specific subject. Supported subject classes include:

1. physical artwork or cultural object;
2. edition instance;
3. component or installation element;
4. digital representation (photograph, scan, mesh, audio/video record, point cloud, texture, radiance field);
5. documentation asset (contract, invoice, catalogue record, condition report, correspondence);
6. derived asset (crop, thumbnail, OCR, caption, embedding, feature vector, index entry, export or training/evaluation sample).

A stable object identifier alone is not sufficient when a permission applies only to one photograph, one scan, one edition instance or one version of a representation. A policy MUST bind the most specific available resource identifier and, where practical, a content digest or immutable version reference.

A resource's classification and policy are separate from its descriptive metadata. Public descriptive metadata does not make its associated image, spatial capture, private provenance source or derived embedding public.

## 4. Operation vocabulary

The following actions are distinct. A grant for one action MUST NOT imply another action unless an explicitly versioned profile rule says otherwise.

| Action family | Examples |
|---|---|
| Acquire and preserve | capture, ingest, preserve, backup, migrate |
| Transform | crop, resize, normalize, transcode, OCR, transcribe, caption, segment |
| Compute | index, embed, feature-extract, classify, compare, infer |
| Access | retrieve, inspect, preview, display, search |
| Distribute | publish, export, share, syndicate, sublicense |
| Research | evaluate, benchmark, train, fine-tune, distill |
| Commercial | advertise, license, sell, commercial-reuse |
| Sensitive disclosure | reveal owner, disclose precise location, disclose restricted cultural context |

The vocabulary MUST distinguish at least index, embed, retrieve, display, publish, evaluate and train. Permission to display a photograph is not permission to index it or create a reusable embedding. Permission to evaluate a model is not permission to train it. Permission to preserve a restricted record is not permission to disclose it.

The profile should map these actions to ODRL actions where a sound mapping exists, and define a documented CSCP extension term only when the ODRL core vocabulary is insufficient.

## 5. Authority is typed, scoped and evidenced

The following roles MUST remain distinct:

- creator / maker;
- legal owner of a physical object;
- physical custodian;
- copyright or other rights holder;
- license grantor acting in a declared capacity;
- institution or collection administrator;
- community-designated cultural authority;
- expert or conservator issuing a professional opinion.

Possession, purchase, creator credit, a valid signature or custody of an object does not automatically establish every other role. The permission evaluator consumes a typed assertion and evidence of capacity; it does not infer competence from a display name, repository role or model score.

A grant record SHOULD bind:
- policy and rule identifiers plus versions;
- exact resource subject and representation/version/digest when applicable;
- assigner and assignee (or an explicitly declared public audience);
- asserted authority role/capacity and supporting credential/evidence references;
- action, purpose, territory/jurisdiction, channel and audience constraints;
- valid-from, valid-until, revocation/status and supersession;
- source assertion and policy digest.

Where a community governs access to sensitive cultural material, the system MUST record the community-defined authority and process rather than infer authority from an individual creator, owner, museum or state institution.

## 6. Deterministic evaluation semantics

The initial CSCP profile uses conservative, deny-by-default evaluation:

1. Resolve the canonical resource identifier and its complete known derivation ancestors.
2. Resolve the requesting actor, declared capacity, purpose, audience, channel and requested operation.
3. Verify policy version, status, validity interval, revocation/supersession state and required authority evidence.
4. Evaluate explicit prohibitions and applicable constraints.
5. Evaluate permissions and duties for the exact subject and operation.
6. Intersect restrictions from every source ancestor and every applicable policy.
7. Return permit only when all required conditions are satisfied; otherwise return deny or a non-disclosing indeterminate result that is treated as deny for protected operations.

A policy conflict MUST NOT be resolved by a model score, majority vote, popularity, issuer count or whichever record is newest. Conflicting assertions remain visible for competent human review.

### 6.1 Monotonic derivation

For a derived asset with source ancestors S = {s1, ..., sn}, the derived asset's effective permissions are no broader than the intersection of the applicable effective permissions of every ancestor, plus any stricter policy attached to the derived asset itself. The only exception is an explicit, independently verifiable declassification or scoped release issued by an authority competent over the exact inherited restriction; it must identify the affected ancestor, output resource, action and conditions. A derivative-local grant without that authority does not override an ancestor policy.

In operational terms:

- each source ancestor must permit the requested transformation;
- the output is born with the restrictions of all applicable ancestors;
- aggregation of multiple sources takes the most restrictive applicable outcome for each action/actor/purpose dimension;
- re-encoding, compression, cropping, resizing, OCR, captioning, embedding, hashing or moving a resource does not remove restrictions;
- a new grant on a derivative cannot silently override an ancestor's prohibition.

Any relaxation requires a separate, explicit authorization from the competent authority for the exact subject, action and derivative scope. The authority and evidence for that relaxation must be independently inspectable.

### 6.2 Unknown and unavailable states

Missing lineage, unknown grantor capacity, incomplete subject scope, stale policy, unavailable policy service, invalid proof, unresolved revocation or conflicting authority MUST NOT be interpreted as permission.

The evaluator may return a reason-coded indeterminate result when it cannot distinguish denial from missing evidence. The enforcing component MUST treat that result as deny for the requested protected operation.

### 6.3 Policy decisions are not credentials

A decision receipt records what a particular CSCP policy snapshot decided for one requested action. It does not confer a transferable permission, establish legal truth or authorize a different resource, operation, purpose, audience or time window.

## 7. Derived-surface enforcement

Policy enforcement MUST cover the whole information path, not only the primary object store.

| Surface | Required control |
|---|---|
| Original file / record | Resource-bound authorization before read or transformation |
| Thumbnails / crops / OCR / captions | Derived ID, source lineage and inherited policy |
| Embeddings / feature vectors | Asset-level permission to embed and retrieve; segregated index; no cross-scope similarity leakage |
| Search index / snippets | Filter before ranking and before result/snippet construction; no sensitive metadata side channel |
| Cache / CDN / previews | Bounded TTL, policy epoch/version, invalidation and authorization on cache hit |
| Export / federation | Preserve policy, lineage, unknown extensions and access classification; fail closed on lossy mapping |
| Logs / traces / analytics | Metadata-minimized audit records; no protected source text, image bytes, precise locations or secret identifiers by default |
| Backup / migration | Preserve access state and lineage; enforce restore-time policy revalidation |
| Training / evaluation | Separate opt-in action grants; track corpus membership and derived model artifacts; publish withdrawal/remediation limits |

A vector embedding is not automatically anonymous. Similarity queries may reveal the presence or attributes of a restricted item. The index and query surface therefore require explicit policy enforcement, tenant separation and adversarial leakage tests.

## 8. Revocation, expiry and correction

Revocation is prospective for future authorization under the declared policy and applicable agreements. It cannot guarantee that already disclosed information is forgotten or that a distributed immutable record is erased.

The implementation MUST distinguish:
- revocation of future use;
- invalidation of cached authorization and derivative availability;
- notification to downstream federated consumers;
- correction/supersession of an erroneous assertion;
- retention required by law, contract, preservation policy or dispute;
- remediation of a prior unauthorized disclosure;
- training-data withdrawal and possible model remediation.

The policy profile MUST publish a measurable revocation-propagation service level. During the propagation window, systems must not claim that the revocation is fully effective everywhere. Protected cache hits MUST revalidate against an allowed policy epoch or fail closed.

For model training, an opt-in grant must be separate from indexing, display and evaluation. Withdrawal stops future use within the declared scope and initiates documented dataset/derivative remediation. The system MUST NOT promise guaranteed unlearning or exact removal from previously trained weights unless a method has been independently validated for that specific model and data.

## 9. Privacy and cultural sovereignty

Public, controlled, restricted and sealed are delivery/access states, not substitutes for the underlying operation-specific policy.

Sensitive location, owner identity, contractual terms and restricted cultural context MUST be protected in:
- payloads and metadata;
- image thumbnails and previews;
- OCR and transcripts;
- embeddings and vector indexes;
- model prompts and explanations;
- error messages, logs and traces;
- exports and public APIs.

The system should support community-defined vocabulary, consent and review workflows without assuming that all cultural materials share the same governance model. A restricted cultural object should not become globally searchable merely because a public surrogate exists.

## 10. Threat model and negative controls

At minimum, test:
- transformation laundering (restricted source converted to a permitted-looking format);
- mixed-source aggregation where one ancestor prohibits the action;
- stale policy and revoked grant replay;
- cross-tenant embedding or nearest-neighbour leakage;
- search snippets that expose sealed fields despite a denied document fetch;
- cache invalidation races;
- prompt injection asking Symthaea to reveal restricted information;
- incomplete lineage or a missing source ancestor;
- valid signature from an issuer with unknown authority capacity;
- export/import that drops unknown restrictions;
- public display with explicit training prohibition;
- training-corpus withdrawal after derived data has been created.

The negative controls must prove that no cognitive model, retrieval score, adapter, cache or transformation can mint or widen a grant.

## 11. Conformance and benchmark programme

Freeze the machine-readable vectors in specs/creative-assets/examples/derived-rights-conformance-vectors.json.

Report each dimension separately:

- policy decision correctness by action and actor class;
- unauthorized permit count (target: zero on the frozen negative corpus);
- false denial and indeterminate rates on reviewed positive cases;
- restriction preservation across all tested derivation edges (target: 100%);
- revocation propagation time at p50/p95/p99;
- cache/index invalidation completeness;
- cross-tenant and side-channel leakage rate;
- policy-evaluation p50/p95/p99 latency;
- lineage completeness and audit-receipt verification;
- import/export round-trip field preservation.

A benchmark pass qualifies only the tested policy engine, adapters, surfaces, fixtures and exact code digest. It does not certify every downstream deployment or settle legal questions.

Initial acceptance targets:
- 100% expected outcomes on frozen positive, negative and ambiguous test vectors;
- zero unauthorized retrieval, embedding, export or disclosure in the frozen adversarial corpus;
- 100% of derived artifacts retain complete source lineage and effective restrictions;
- 100% of stale/revoked authorization fixtures reject after the declared revocation SLA;
- no model-generated or inferred permission accepted by an enforcement boundary.

## 12. System ownership and integration

- **Mycelix / CSCP:** canonical policy, identity, authority assertions, permission decisions, revocation state and event lineage.
- **Symthaea:** can request policy-filtered retrieval and explain reason-coded outcomes. It cannot grant, widen or reinterpret permissions. Prompt context and embeddings must obey CSCP decisions.
- **Sol-Atlas:** binds rights to captures, geometry assets, scenes, coordinate/location data and all generated spatial derivatives.
- **Symtropy:** binds rights to sensor traces, condition reports, simulation inputs/outputs and conservation publications.
- **Independent verifier:** replays policy and lineage fixtures from frozen inputs and reports exactly what was verified.

No downstream system should create its own competing rights authority. Each adapter must consume a versioned CSCP decision contract and return auditable evidence of enforcement.

## 13. Pilot and release gates

Begin with synthetic records and public-domain or individually permissioned assets. Do not use restricted cultural records in a public pilot merely to test access controls.

Before a real institutional or community pilot:
1. review the permission vocabulary and mapping with artists/rightsholders;
2. agree how community-designated authority is represented and who can update it;
3. test all derived surfaces, including caches and vector indexes;
4. document revocation propagation and retention exceptions;
5. conduct an independent negative-control review;
6. publish the precise scope and limitations of the pilot.

The first pilot should demonstrate that a permission can be expressed once, propagated through each permitted transformation, enforced at every read/write/search boundary, and revoked without falsely claiming retroactive erasure.