# Medicinal Botanical Provenance and Benefit-Sharing Contract v1

**Status:** proposed provenance/governance contract; not legal advice, botanical identification, a permit, or product authorization.  
**Parent:** [Open home-health aids and medicine dossier PR #5028](https://github.com/Luminous-Dynamics/mycelix/pull/5028).

## 1. Design principle

Mycelix coordinates **who supplied what, under which declared rights, with which scoped evidence and changes**, while leaving scientific and legal judgments with qualified owners.

The core distinctions are:

`plant provenance != authenticated species != safe material != clinical evidence != authorized product != released batch`

A Holochain action, signed attestation, DHT record or digest proves only the properties the verifier actually checks. It does not make a species assertion true, prove a study worked, grant access rights, establish consent, or release a medicine.

Growing a plant does not automatically authorize commercial bioprospecting, access to associated knowledge, transfer to a manufacturer, public disclosure of its location or downstream IP claims.

## 2. Separate subjects and permissions

Keep these subjects distinct:

- `GardenProfileRef` — opaque site ID and reviewed coarse locality token; no public household coordinates by default.
- `BiologicalMaterialRef` — plant/propagule/specimen/harvest lot identity with declared origin and custody lineage.
- `IdentificationAssessmentRef` — taxonomic identifier, method, evidence/voucher reference, confidence/limits and currentness.
- `KnowledgeAssetRef` — reference to knowledge contributed under explicit disclosure and reuse policy; do not store sensitive payloads on public DHT entries by default.
- `RightsAssessmentRef` — reviewed scope for collection, access, research, export, commercial use and disclosure.
- `ConsentDecisionRef` — what participant/community authorized, purpose, recipients, term and withdrawal/review procedure.
- `BenefitSharingAgreementRef` — parties, material/knowledge scope, governance authority, effective period, monetary/non-monetary benefits, accounting, dispute process and delivery evidence.
- `QualityEvidenceRef` — external lab/material assessment tied to a sample and custody identity.
- `QualifiedProductRef` — exact product/site/process/jurisdiction scope from an authorized owner.
- `BatchDispositionRef` — exact finished-product lot and external disposition authority; never substitute a raw-plant lot ID.

Every permission has its own subject and purpose. Generic `consent=true` cannot authorize all downstream uses. Permission to grow or share seedlings is not permission to publish location, reveal knowledge, export material, conduct research, train a model on restricted content or commercialize a derivative.

## 3. Rights/disclosure state machine

Proposed explicit states:

- `Draft` — local preparation only.
- `ReviewRequired` — legal, taxonomic, ecological or rights review missing/stale.
- `ConsentPending` — one or more required parties have not granted scoped consent.
- `AuthorizedForDeclaredPurpose` — exact activity and recipients are allowed by current reviewed evidence; no broader permission follows.
- `Restricted` — material/knowledge can be used only within named limits.
- `Suspended` — objection, expiry, safety concern, permit issue or governance dispute blocks new action.
- `Withdrawn` — no new use is permitted under applicable rules; downstream actions and legal obligations still need case-specific review.
- `Expired` / `Superseded` — no longer current; historical decision remains auditable.
- `NotApplicableWithRationale` — only for an explicitly scoped gate, with evidence and rationale.

Unknown or malformed permission fails closed. Software must not decide whether an activity legally requires a permit; it records the reviewed determination and source.

## 4. Provenance graph

Minimum chain:

`propagation source -> garden/site ref -> cultivation episode -> harvest event -> raw botanical lot -> sampling/identity/quality evidence -> custody transfers -> recipient intake decision`

For qualified research or regulated manufacturing, add separate subjects:

`research/production work order -> authorized process generation -> input lots -> execution evidence -> output lot -> analytical/quality disposition -> external release decision`

Every edge binds immutable IDs and generations. Missing edges produce `Incomplete` or `Unknown`. A new harvest is a new lot even at the same site; processing creates a new subject rather than mutating its input. Failed/contaminated lots and later corrections remain in history.

## 5. Minimal contract sketches

### `BotanicalMaterialLotV1`

Required semantics: opaque lot ID and schema version; declared taxon and taxonomic-assessment reference (a name is not identity proof); descriptive plant part/form (not a preparation recipe); propagation and cultivation references; generalized origin and rights assessment; harvest window with privacy-sensitive precision kept local where unnecessary; quantity/unit/measurement reference; custody events, package/seal state if applicable, storage observations; sample/test references; explicit claim ceiling and limitations.

States may include `Reported`, `IdentityPending`, `QualityPending`, `AcceptedForDeclaredUse`, `Rejected`, `Quarantined`, `Recalled`, `Exhausted`, `Unknown` and `Superseded`. `AcceptedForDeclaredUse` requires a named approving party, exact destination/use scope and required identity/quality references. It means only accepted for that intake/use—not clinically safe/effective.

### `KnowledgeUseGrantV1`

Bind opaque knowledge ID, rights-holder/community governance reference, recipient, exact purpose, disclosed fields, permitted transformations, retention term, redistribution/model-training/commercial-use scope, attribution, benefit agreement, expiry and dispute/withdrawal route.

Sensitive, sacred, confidential or collectively held knowledge defaults to `NotDisclosedToNetwork`. Prove a permitted action without copying the knowledge to every node. Public pseudonymous metadata can still identify people when combined with locality or rare-species details; review disclosure risk before publication.

### `BenefitDeliveryReceiptV1`

Record the agreed benefit class, recipient/governance authority, due date, delivery evidence, acceptance/dispute state and reconciliation reference. A signed promise is not a delivered benefit. Payment finality belongs to the authorized financial/payment subsystem; Mycelix stores a cross-reference/receipt.

## 6. Event-admission and adversarial checks

The initial Rust deliverable should be a pure domain evaluator before a zome/UI integration. Required fail-closed cases:

1. duplicate lot/event/agreement IDs or cross-scope collisions;
2. unknown species determination represented as authenticated identity;
3. transfer to a recipient outside the rights/consent scope;
4. commercial use attempted on a non-commercial research grant;
5. knowledge access inferred from material access;
6. expired/revoked/suspended decision reused as current authority;
7. malformed consent metadata or omitted required party;
8. changed species/site/lot/process generation accepted under an old receipt;
9. duplicate/derivative observations counted as independent identity or quality confirmations;
10. failed/quarantined material counted as available stock;
11. raw botanical lot ID substituted for a finished-product batch;
12. public record leaks exact home coordinates, patient data or restricted knowledge;
13. benefits shown as fulfilled without a delivery/acceptance receipt;
14. syntactically valid digest with unverified bytes called authenticated evidence;
15. manufacturer/lab/regulator assertion forged by a garden participant;
16. model/simulation success substituted for authorization or quality release.

Test replay and independent verification. Preserve negative controls rather than weakening them to clear CI.

## 7. South African consent, conservation and research

For South African use, have competent counsel and the relevant authority determine current applicability before collection for research, transfer/export or commercial bioprospecting. The National Environmental Management: Biodiversity Act addresses sustainable use of indigenous biological resources and fair/equitable benefit-sharing; DFFE publishes BABS permitting information. Check current regulations and amendments: a draft is not law, and this document does not assert that every home garden requires a permit.

- [National Environmental Management: Biodiversity Act 10 of 2004](https://www.gov.za/ss/documents/national-environmental-management-biodiversity-act-0)
- [DFFE BABS clearing house and permitting information](https://www.dffe.gov.za/bc_projectsprogrammes_babs)
- [Government Gazette: BABS regulations](https://www.gov.za/documents/national-environmental-management-biodiversity-act-regulations-bio-prospecting-access-and-0)
- [SANBI medicinal garden and safety notice](https://www.sanbi.org/gardens/free-state/tours/medicinal-garden/)

A community may refuse, keep knowledge private, place conditions on use and challenge benefit calculations/delivery. Never infer consent from historical use, public mention, social-media posts or the fact a plant grows on land.

## 8. Repository ownership

- **Mycelix:** consent, governance, provenance references, scoped sharing, custody events and benefit audit.
- **Symthaea:** advisory research/design and evidence organization; cannot mint legal rights, declare clinical truth or approve its own outputs.
- **Symtropy:** bounded deterministic infrastructure/supply/physics scenarios using externally qualified input states.
- **Sol Atlas:** evidence-bound spatial/climate/environment overlays; no inference of identity, land permission, soil/water safety or medicine suitability.
- **Mycelix Health:** health evidence, reviewed botanical/quality gate semantics and regulated-product boundaries. Keep that domain-specific authority there rather than deleting it or absorbing it into a general-purpose repository.

## 9. Acceptance criteria

Ready for implementation when tests prove: every action is purpose/recipient-scoped; sensitive knowledge defaults to local/restricted; rights cannot be inferred from available data; raw and transformed lots remain distinct; failures/withdrawals remain auditable; and no provenance receipt is presented as clinical, scientific, regulatory or batch-release proof.
