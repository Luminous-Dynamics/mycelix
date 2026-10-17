# CSCP Creative Asset Passport and Rights Graph

**Status:** Research/architecture proposal; not a legal registry or production implementation.
**Date:** 2026-10-17
**Parent:** [Creator Sovereignty Control Plane RFC](CREATOR_SOVEREIGNTY_CONTROL_PLANE_RFC.md)
**Related workflows:** [Deal-to-Cash Model](CSCP_DEAL_TO_CASH_MODEL.md) · [Creator Resilience Model](CSCP_CREATOR_RESILIENCE_MODEL.md)

## Executive thesis

For music artists, the creator's most valuable durable asset is not an account, a post, or a follower count. It is a catalogue of creative works, recordings, performances, visual assets, collaborations, licenses, and versions—plus evidence for who contributed, what was agreed, how an asset may be used, and what income has actually been reported.

The proposed **Creative Asset Passport** is a private-first, exportable record that links those facts without pretending that software can adjudicate copyright ownership. It should let a creator answer:

- Which exact composition, master recording, video edit, live version, or clip is being discussed?
- Which people contributed, what roles did they perform, and which credit/split records have been explicitly approved?
- Which terms and rights schedules applied to this release, ad campaign, remix, livestream, sample, or AI-assisted derivative?
- What uses are authorized, for which territory, period, channel, and purpose?
- What identifiers and registrations exist, which are missing, and which external source supplied them?
- What royalty statement or cash event refers to the work or recording, and what is still only expected, disputed, or unreconciled?
- Can the creator export the complete supported catalogue and continue using it if a distributor, label, platform, or service disappears?

This is not another music distribution service, royalty collection society, blockchain certificate, or ownership adjudicator. It is the evidence and coordination layer connecting existing tools and institutions.

## Research findings and what they do—and do not—show

### Rights metadata and asset identity are structurally distinct

The official ISWC authority states that an ISWC uniquely identifies a musical work, but it does not identify recordings, establish ownership, or encode the rights shares. The IFPI International ISRC system identifies a particular sound recording or music-video recording. An ISRC is not an identifier for a composition, performer, release product, or rights owner.

Sources:
- [ISWC: what it does and does not do](https://www.iswc.org/iswc)
- [IFPI ISRC FAQs](https://isrc.ifpi.org/faqs)
- [IFPI: using ISRC](https://isrc.ifpi.org/why-use-isrc)

This creates an important product invariant: a title or creator name is not a stable canonical identity, and an identifier is not proof of rights. The passport must model related but distinct objects: musical work/composition, recording/master, version/edit, release/product, visual/audio-visual asset, and license/use grant.

Do not invent an ISRC, ISWC, ISAN, IPI, distributor ID, or collecting-society registration number. IDs remain optional until assigned by the relevant authorized scheme; external IDs need their issuing authority, source, and observed timestamp.

### Music is a large global rights and revenue ecosystem

IFPI's March 2026 *Global Music Report* says global recorded-music revenues grew 6.4% to US$31.7 billion in 2025, with streaming revenues above US$22 billion. CISAC's 2025 *Global Collections Report* says total royalties collected across its creative-industry network reached EUR13.97 billion in 2024; its data covers 228 member societies in 111 countries and multiple repertoires, not recorded music alone.

Sources:
- [IFPI Global Music Report 2026](https://www.ifpi.org/global-music-report-2026-global-recorded-music-revenues-grow-6-4-as-record-companies-drive-innovation/)
- [CISAC Global Collections Report 2025](https://www.cisac.org/cisac-global-collections-report-2025)

These figures show the scale and complexity of the ecosystem. They are **not** estimates of the market available to this software, and they do not prove that an asset passport will increase an artist's income.

### Unauthorized digital replicas create a concrete consent boundary

The U.S. Copyright Office's July 2024 report on digital replicas recommended that Congress consider a federal right against knowing distribution of unauthorized digital replicas. That document is a policy recommendation; it is not by itself proof of a new federal statute being enacted. Legal protection differs by jurisdiction.

Source: [U.S. Copyright Office, Part 1: Digital Replicas](https://copyright.gov/ai/Copyright-and-Artificial-Intelligence-Part-1-Digital-Replicas-Report.pdf).

The product requirement is broader than copyright: permission to store, analyze, publish, distribute, train a model on, clone a voice/likeness, create a derivative, and license a commercial use are distinct purposes. One broad “consent” checkbox must not authorize all of them.

## Paradigm shifts

### 1. From files and accounts to a rights-aware asset graph

Represent the relationships among work, recording, version, asset, contributor, agreement, license, publication, event, and cash/royalty evidence. Make these relationships portable and inspectable rather than hiding them inside separate distribution, social, contract, and storage tools.

The graph records claims, evidence, and approvals. It does not convert a contributor credit, uploaded file, ISRC, ISWC, or blockchain receipt into a legal determination of ownership.

### 2. From “copyright checkbox” to scoped, revocable permissions

A grant must describe the purpose and scope of the allowed action. Suggested fields:
- who grants permission, to whom, and in which capacity;
- the asset and exact version covered;
- permitted purposes (store, analyze, publish, distribute, advertise, train, generate derivative, clone voice/likeness, license, monetize);
- territory, channels, term, exclusivity, attribution, compensation reference, and required disclosures;
- evidence reference to the approved agreement/consent;
- status: proposed, approved, active, expired, revoked for future use, disputed, or superseded;
- the event/time from which permission became effective and the history of corrections.

A revocation stops future access/use to the extent controlled by the product and applicable agreements; it cannot promise to retract copies already downloaded, media already published by third parties, or immutable records. The product must state these limits rather than overpromise deletion.

### 3. From credits-as-text to explicit contribution and split records

Separate at least:
- creative roles/credits;
- agreed monetary split or other contractual compensation;
- the relevant rights pool (for example, composition/publishing vs. master recording), territory, and term;
- approval status and evidence;
- actual royalty statements or payment events.

A creator may be credited without owning a share; ownership and compensation can be governed by different agreements. The system must not infer a split from play time, time spent, AI attribution, project membership, or a credit string.

Where a particular pool is declared complete, shares for that pool must validate against its agreed allocation rule (typically 100%, with explicit handling of administrator/label shares and other pools). Do not sum shares across different rights pools or territories. A mismatch becomes an unresolved exception—not an automatic normalization.

### 4. From AI provenance theater to usable evidence lineage

For AI-assisted creative work, record only what is known and authorized: tool/model/version if supplied, human-selected inputs, source asset references, the creator's declaration of contribution/approval, and any separate consent for voice/likeness or training use.

Do not claim that a cryptographic hash proves authorship, originality, training provenance, or legal ownership. A hash can help detect whether bytes changed; identity, rights, and authority require separate evidence.

### 5. From catalog lock-in to portable artist infrastructure

Support canonical local IDs with optional external identifiers and fully documented exports. Where permitted, map to official external identifiers without replacing them. Keep confidential agreements, identity documents, raw private communications, banking details, and private fan data out of public distributed stores.

Mycelix may later host permissioned attestations for claims, credits, or consent records, but the default asset passport should work locally first. Use a distributed layer only when a concrete cross-party coordination need justifies its privacy, durability, correction, and governance trade-offs.

## Proposed data model

This is a minimal conceptual schema, not final Rust or Holochain types.

- **CreativeWork**: local canonical ID, title aliases, type, language, creators/roles, optional ISWC or external work IDs with issuer/source.
- **Recording**: local ID, parent work links, master/sound recording reference, version/mix/live/studio status, optional ISRC and issuing source.
- **CreativeAsset**: local ID, asset type, content digest if available, storage reference, visibility, creator-selected version label, created/observed time, optional parent/derivative link.
- **Release**: local ID, grouping of recordings/assets, distributor, territories/dates, optional release identifier, publication state, linked commercial terms.
- **ContributionClaim**: asset/work/recording reference, party, role, claim source, contributor confirmation, dispute/supersession history.
- **SplitAgreement**: right-pool ID, territory, effective interval, parties and exact agreed shares/amounts, source terms version, approvals, unresolved/conflicting status.
- **UsageGrant**: grantor, grantee, asset/version, purposes, territory, channels, start/end, exclusivity, compensation reference, attribution/disclosure rules, approval and revocation history.
- **AIUseDeclaration**: source/input references where known and authorized, declared tool/model metadata, human approvals, training/voice/likeness scopes, and unknown fields.
- **RoyaltyStatement**: reporting entity, period, external identifiers, uses/units, gross/adjustments/net/currency as reported, source evidence, import status, and reconciliation to actual cash.
- **ExternalIdentifier**: namespace/scheme, value, issuer/registry, subject local ID, resolution status, source, observation time, and possible collision/merge references.

Each fact has a provenance class: creator-declared, counterpart-approved, imported from an external source, independently validated against that source, derived, disputed, or unknown. These states must not be collapsed into a single “verified” badge.

## Release/readiness contract

A release can still be drafted while data is incomplete. The system must distinguish “unknown” from “cleared.” It should surface a release/readiness report that lists unresolved conditions before a creator chooses to publish or license the asset.

Recommended gates:
- no unresolved split agreement may be represented as a unanimously approved split;
- no missing external identifier is silently fabricated;
- no asset's recording identifier is mistaken for a work identifier;
- no usage grant expands beyond its approved asset, purpose, territory, term, or channel;
- no AI training or digital-replica permission is inferred from permission to analyze or publish a source;
- no generated royalty estimate is labeled as actual cash received;
- no rights or credit claim is silently overwritten; corrections create a superseding event and preserve prior history;
- no conflict is auto-resolved by selecting the most recent claim without the relevant authority/evidence.

These controls reduce coordination risk, but they do not guarantee legal clearance. A release-readiness warning is a workflow aid, not a legal opinion or substitute for competent rights administration.

## Benchmark programme

Targets below are proposed internal gates. They are not external standards or achieved results.

| Dimension | Proposed benchmark | Test contract |
|---|---|---|
| Asset identity | 100% preservation of the distinction between work/composition, recording, version, release, and audiovisual asset. | Adversarial fixtures where titles/artists collide; same recording in multiple formats; different live/remaster/edit versions. |
| External ID integrity | Zero fabricated or re-used identifiers; 100% of imported identifiers retain scheme, subject type, source, and observation time. | Test ISRC/ISWC type confusion, duplicate/missing IDs, malformed values, and collision/merge history. |
| Split arithmetic | 100% exact agreement with golden integer/rational split fixtures. | Each right pool is tested separately; conflicting/missing shares fail closed; never silently normalize to 100%. |
| Approval correctness | Zero “approved” split or grant without all required explicit approvals for the declared workflow. | Wrong actor, stale terms version, amended terms, duplicate/replayed approvals, revoked authorization. |
| Scope enforcement | Zero rights grants exceeding the approved purpose, territory, channel, term, or asset version. | Test store-vs-publish, analysis-vs-training, publishing-vs-digital-replica use, expiration, and future-use revocation. |
| Provenance | 100% of claims identify source and evidence class; zero source-free material factual claims in benchmark reports. | Tampered content digest, changed source reference, disputed claim, unknown contributor, source unavailable. |
| Royalty accounting | 100% of reported statement and cash figures remain distinguishable; zero expected royalties shown as cash received. | Statement estimates, delayed/partial statements, duplicate imports, withholding/fees, refunds, multiple currencies, corrections. |
| Portability | Zero unexplained loss of supported asset, permission, contributor, identifier, or supersession records on round-trip export/import. | Unicode, version migration, expired/revoked grants, optional IDs, disputed claims, partial imports. |
| Privacy | Zero private contracts, identity proofs, unreleased media, banking details, or fan PII exposed through public projections by default. | Serialized output inspection, projection tests, access revocation, retention/deletion behavior. |
| Creator comprehension | Median ≥4/5 for whether a creator can explain the current rights/credit status and missing evidence in a design-partner study. | Report distributions, uncertainty, and specific misunderstandings; no badge-based vanity score. |
| Workflow value | Exploratory target: reduce creator-reported effort spent reconstructing rights/credits/usage terms by ≥30%. | Four-week baseline and 8–12-week pilot, same task diary, include setup/import overhead; do not claim causal impact without an appropriate comparison. |

## Adversarial fixture set

1. Two compositions share a title and artist alias but have different external IDs. Never auto-merge by text.
2. One composition has three recordings, including a live version and a remaster. Do not assign the same identity to different recordings without the applicable official rule.
3. An ISRC is present but the corresponding composition/ISWC is missing. Keep the recording identified and the composition link unresolved; do not invent a work code.
4. An ISWC is present but participants disagree on their shares. Identification must not be mistaken for ownership proof.
5. Writer shares total 97%, while master splits separately total 100%. Keep the pools distinct and flag the incomplete writer pool.
6. One co-writer approval references an earlier split version. It does not approve the current version.
7. A license permits online video posting but not model training or voice cloning. The latter actions remain unauthorized.
8. A sponsor uses a campaign asset outside the approved term or territory. Flag the scope mismatch; do not rewrite the grant.
9. A creator revokes consent for future voice-cloning use. Stop future product-controlled use within the stated scope; disclose limits on copies already made.
10. A content digest remains identical but the ownership claim is disputed. Hash equality must not resolve the ownership dispute.
11. A royalty statement reports expected royalties but no cash receipt exists. Preserve it as statement data, not cash income.
12. A release has unresolved split conflicts. Display a blocker/warning and affected pool; do not silently pick a winner.
13. A shared stem/sample asset is included in a derivative. Require source/permission references as applicable and preserve the parent-child link.
14. An identifier is later corrected or merged in an external registry. Preserve the old reference and record the authoritative replacement/merge evidence.
15. A creator changes distributor. Keep the local canonical asset graph and external identifiers intact; update distributor mapping without taking over ownership.
16. An AI music tool assisted with arrangement but training/likeness permissions are absent. Record unknown/declared provenance and do not infer authorization.
17. A contributor is credited but explicitly has no ownership share under the agreed terms. Preserve credit separately from ownership.
18. A catalog export omits revoked grants and disputes. Round-trip verifier must detect missing supported history or clearly record a documented export policy limitation.

## Responsibilities across the four projects

- **Mycelix:** later support portable, scope-limited claims and consent/approval histories where cross-party trust needs it. Never publicly expose private contracts, unreleased works, private IDs, or fan data by default.
- **Symthaea:** help compare source evidence, flag inconsistencies, summarize rights/credit status, and propose missing-data questions. It must not judge ownership or resolve a dispute autonomously.
- **Symtropy:** later generate interactive live experiences that reference approved asset versions and their allowed use. A game's or stage's access to a file is not a license to reuse it elsewhere.
- **Sol-Atlas:** later link approved catalog/performances to event, venue, and tour plans. Geospatial planning does not grant rights to publicly identify fans or participants.
- **Melothaea / Mycelix-Music:** future music-specific tooling may use the passport as a metadata/rights contract, but music generation, instrument/style capability, release management, and legal rights are distinct concerns.

## Proposed delivery sequence

1. Publish this contract and agree on the distinction between works, recordings, releases, rights pools, and usage grants.
2. Build a local-first schema and validator with synthetic fixtures; use optional external IDs, never fabricated ones.
3. Add exact split arithmetic, explicit approval/version history, and conflict states.
4. Add rights grants with distinct purposes/territories/terms and AI/voice-likeness scopes.
5. Add royalty statement ingestion as a separate data source, while preserving expected statement amounts vs actual cash.
6. Implement complete export and independent round-trip/manifest verification.
7. Only then consider Mycelix distributed claims or Symthaea-powered advisory analysis for specific cross-party needs.

## Go/no-go rule

Do not release an asset-passport workflow if it can silently conflate credits and ownership, a work and a recording, an approval of older terms with approval of current terms, an external identifier with legal rights, or an expected royalty with money received.

