# Art Market Research and Benchmark Programme
**Status:** Research/benchmark proposal v0.1  
**Parent:** CSCP Creative Asset Passport and Physical Artwork Extension  
**Purpose:** compare Luminous with current art-market workflows and define credible, task-specific routes beyond current baselines  
**Claim discipline:** targets below are hypotheses; no benchmark result or market impact is claimed by this document

## 1. Market problem and product thesis

The 2026 Art Basel & UBS Global Art Market Report estimates 2025 global sales at US$59.6B across 41.5M transactions. Online-only sales fell to US$9.2B, or 15% of the market. This is a warning against treating an online marketplace as the whole solution: high-value transactions still depend heavily on physical inspection, trusted relationships, documentation, and institutions.

Artsy's 2025 collector survey reports that 69% of surveyed collectors had hesitated to buy because of insufficient transparency and 60% wanted better price transparency. This is a vendor survey, not a census; treat it as a directional signal, not a market-wide prevalence estimate.

A Management Science study using auction-catalogue data found that documented provenance dimensions were associated with changes in sale probability, prices, and repeat-sale returns. Its authors discuss endogeneity: better-documented works may differ systematically from poorly documented works, so the observed associations are not guaranteed causal returns from a passport.

**Product thesis:** build the evidence, identity, rights, spatial, lifecycle, and coordination infrastructure that galleries, artists, collectors, museums, insurers and marketplaces can use. A marketplace is a possible downstream client, not the core product.

## 2. Industry comparison: where to compete and where to integrate

| Existing approach | Current strength / evidence | Gap or Luminous opportunity |
|---|---|---|
| Artlogic | More than 6,000 galleries, artists and collectors; over 12,000 professionals; stores/shares over 10M artworks. Its 2026 CoA tracking beta records certificate status, origin/ownership/delivery notes and attachments. | Do not compete first on gallery CRM/inventory UI. Offer portable claims, standards adapters, cross-system verification, independent evidence lineage and richer lifecycle/spatial records. |
| ArtCheck | Documentary provenance screening across databases, archives and institutional sources; issues a numbered provenance verification report. | Treat expert due diligence as a partner/issuer. Passport should preserve scope, source set, time, limitations and report digest, not claim to replace human researchers. |
| Artsy | Established discovery/marketplace and collector-facing online channel. Survey research indicates transparency remains a purchase barrier. | Complement marketplaces with portable identities, rights-aware data and evidence-rich discovery; do not assume a new marketplace can quickly acquire liquidity. |
| Auction-price analytics | Hedonic models and repeat-sales analysis are established; South African studies now compare non-parametric ML approaches. | Compete on calibration, data lineage, local/regional coverage, long-tail artists and explanation—not a black-box price point. |
| Museum collection portals | Large structured collections and public APIs exist; The Met exposes over 470,000 records and CC0 public-domain data/images where rights permit. | Build lawful, source-preserving adapters and cross-collection search; do not scrape rights-restricted images or imply the whole dataset is public domain. |

## 3. Architecture differentiation

The Luminous system should be deliberately federated:

- Mycelix/CSCP: stable identity, assertions, credentials, rights, privacy, consent, claim/event lineage, creator economics.
- Symthaea: explainable multimodal discovery, uncertainty-aware evidence analysis, discrepancy detection, research assistance.
- Sol-Atlas: metric geometry, scene graphs, object components, coordinate transforms, spatial provenance and uncertainty.
- Symtropy: environmental exposure ledgers, calibrated condition/transport simulations, deterministic replay and physical-model receipts.

Build standards adapters before proprietary schema proliferation:
- Object ID for minimum object identification;
- CIDOC CRM / ISO 21127:2023 for event/relationship semantics;
- LIDO for collection interchange;
- W3C VC 2.0 / Data Integrity for signed assertions;
- PROV-O for derivation and agent/activity lineage;
- IIIF for image/manifest delivery where applicable;
- PREMIS/OAIS and persistent identifiers for preservation;
- open spatial formats with explicit coordinate frames and calibration.

Cryptographic integrity proves that a record has not changed and who signed it; it does not prove the claim is true, the issuer is competent, or the legal title is valid.

## 3.1 Cultural governance and permission interoperability

The physical-artwork extension must support community-defined protocols without assuming that all cultural objects share one governance model. For Indigenous data governance, the CARE Principles (Collective Benefit, Authority to Control, Responsibility, Ethics) complement technical FAIR interoperability; they are not a machine-readable license. Where communities choose Local Contexts TK/BC Labels, preserve the permanent label identifier, label title, community-customized description, language/translation, provenance and source status.

**Critical semantic boundary:** a community label communicates governance context and expectations; it is not automatically a universal executable permission, copyright determination or proof of legal authority. The system must retain the community's own decision process and require an exact, operation-scoped decision from the appropriate community-designated authority where such authorization is required. Never infer permission to index, embed, train, commercialize or disclose sensitive context from public access, museum custody or a label alone.

References:
- [Global Indigenous Data Alliance — CARE Principles](https://www.gida-global.org/careprinciples)
- [Local Contexts — TK/BC Labels](https://localcontexts.org/labels/about-the-labels/)
- [Local Contexts — community label workflow](https://localcontexts.org/support/working-with-labels/)

## 4. Benchmark ladder

### A. Passport integrity and interoperability

**Baselines:** Object ID checklist, existing collection exports, W3C VC 2.0 verifier behavior, and the current CSCP asset-passport invariants.

**Measures:** schema conformance, round-trip field preservation, provenance completeness, digest verification, signed-claim verification, revocation/supersession handling, contradiction preservation, selective disclosure leakage.

**Proposed targets:**
- 100% of positive fixtures accepted and every negative fixture rejected with stable reason codes.
- 100% of provenance assertions carry an issuer/role, asserted time, evidence links or an explicit unknown/unsupported state.
- Zero silent loss of unsupported fields across import/export.
- Zero silent reconciliation of conflicting attribution, ownership, custody or rights claims.
- Zero private-location/owner/contract leakage in public projections.
- 100% detection of altered payloads and event-lineage breaks in the conformance suite.

### B. Multimodal art discovery and semantic retrieval

**Baselines:**
- BM25/text metadata retrieval;
- CLIP/OpenCLIP or SigLIP image-text embeddings;
- DINOv2 visual embeddings;
- hybrid metadata + visual ranking;
- curated human-curator relevance judgments.

ArtBench-10 contains 60,000 images across 10 styles and can serve as a narrow style-classification baseline, not a universal measure of art understanding. The Met's Open Access API offers a large lawful seed corpus for public-domain works; individual image and metadata rights still need checking.

**Evaluation design:**
- 1,000+ rights-cleared query/artwork pairs with curator/artist-authored relevance labels;
- artist-disjoint, institution-disjoint and time-held-out partitions;
- multilingual and cross-cultural coverage;
- near-duplicate and image-crop leakage checks;
- separate tasks for style, iconography, material, theme, artist similarity, historical context and visual resemblance.

**Metrics:** Recall@K, nDCG@K, mean average precision, precision at curator review budget, cross-language degradation, artist/institution holdout, calibration and abstention.

**Stretch target:** ≥5% relative nDCG@10 improvement over the strongest matched open baseline on held-out artists/institutions, while maintaining or improving calibration and showing no material regression for underrepresented regions/styles. This is a target, not a result. A model should explain why two works were retrieved and avoid declaring cultural meaning from visual resemblance alone.

### C. Provenance and attribution evidence analysis

**Baselines:** manual expert chronology review, rules for date/owner/event consistency, and source-document retrieval.

**Tasks:** OCR/document extraction where licensed, entity resolution, chronology checks, missing-link discovery, evidence clustering, and contradiction surfacing.

**Metrics:** claim extraction precision/recall by claim type, chronology error detection, false contradiction rate, evidence-link coverage, expert correction time, and abstention rate.

**Proposed gates:**
- ≥98% precision for any machine-generated high-risk ownership/custody discrepancy alert before showing it as a priority review item;
- ≥90% recall on a frozen, expert-adjudicated set of material chronology contradictions;
- 100% machine-generated assertions visibly labelled as candidate/unverified until authorized issuer review;
- zero automatic “authentic” / “forged” / “legally owned by” decisions.

Do not train/evaluate on the same provenance text duplicated across auction catalogues, archives and collection pages. Partition by artwork and source institution.

### D. Indicative valuation and market intelligence

A 2025 study of the South African fine-art auction market used 43,354 auction lots, 26,486 sold, and compared random forest, gradient boosting, CNN, hedonic OLS and quantile regression approaches. It reported random forest as the strongest single-point model and gradient boosting quantile regression as the strongest quantile approach. A 2023 South African study used 44,912 auction-lot observations from 2009–2021 to analyze quantile hedonic prices.

This is a useful regional research baseline, but the underlying full dataset may not be public. We must obtain lawful data and author permission before claiming exact reproduction.

**Required baseline ladder:**
1. median/artist-era/medium baseline;
2. hedonic OLS with artist/time/auction effects;
3. random forest;
4. gradient boosting with quantile loss;
5. image-only CNN baseline;
6. hybrid model with provenance, catalogue, visual, temporal and market context.

**Evaluation design:**
- chronological holdout (train earlier, test later);
- artist-disjoint and auction-house-disjoint holdouts;
- separate sold/unsold selection process;
- currency, inflation and buyer-premium/hammer-price definitions fixed;
- avoid repeat-lot leakage and duplicate catalogue records;
- confidence intervals and quantiles, not a single predicted price.

**Metrics:** MAE and median absolute error, pinball loss, quantile coverage, interval width, calibration by price tier/artist/medium/region, top-decile error, drift and missing-data sensitivity.

**Stretch target:** ≥5% relative MAE improvement against the best lawful matched baseline, with 90% predictive interval coverage between 87–93% on chronological and artist-held-out tests. If the model cannot meet coverage, widen intervals or abstain. This remains an indicative research estimate, never investment advice or a substitute for an appraisal.

### E. Spatial reconstruction and digital twins

**Baselines:** calibrated photogrammetry / SfM-MVS, depth or structured-light scans, laser scans where feasible, and separately evaluated NeRF/Gaussian-splat view synthesis.

A 2026 SHREC cultural-heritage benchmark reports that global Chamfer Distance can misrank reconstructions by favoring smooth surfaces while losing inscriptions, tool marks and high-frequency relief. Sol-Atlas should therefore evaluate feature preservation, not just global averages.

**Metrics:** point-to-plane RMSE, symmetric Chamfer, F-score at declared tolerances, normal consistency, registration/scale error, feature-aware detail score, completeness/occlusion, uncertainty coverage, color error only when calibrated, runtime and capture cost.

**Stretch target:** improve feature-weighted fidelity by ≥10% over the strongest matched baseline at equal or lower capture cost, without degrading global geometric accuracy beyond a predeclared tolerance. Use independent metrology as reference.

### F. Conservation and lifecycle

There is no single universal condition benchmark across all artwork materials and mechanisms. Create an expert-adjudicated, rights-cleared collection of repeated observations and sensor histories.

**Baselines:** current institution thresholds, transparent dose/exposure summaries, and conservator condition assessment.

**Metrics:** alert precision/recall, false alerts per object-month, detection delay, calibration, abstention coverage, object/site transfer, sensor drift and data completeness.

**Gate:** Symtropy first reproduces measured exposure summaries exactly against independent reference calculations. Material-specific deterioration claims require independent material science parameters, calibration objects, held-out objects/sites and conservator review.

### G. Artist and institutional outcomes

Measure in a pilot rather than infer impact from software:
- registration and documentation time;
- provenance completeness and time spent resolving missing evidence;
- collector diligence time;
- number of qualified inquiries and new buyer relationships;
- licensing income and payment delay where creators voluntarily supply records;
- artist control over use, permissions and disclosure;
- accessibility and participation by emerging artists/institutions.

Do not equate higher sale prices with a healthier art ecosystem. Include creator net proceeds, access, cultural stewardship and rights retention.

## 5. Paradigm-shift hypotheses

1. **From certificates to evidence graphs:** claims remain source-linked, scoped and disputable.
2. **From object catalogues to lifecycle passports:** identity connects physical state, rights, events, representations and spatial models.
3. **From single-market databases to federated interoperability:** institutions retain authority and data custody while exchanging verified assertions.
4. **From AI “authentication” to evidence triage:** AI finds inconsistencies and missing records; accountable experts make attribution decisions.
5. **From price points to calibrated market ranges:** account for uncertainty, selection bias and sparse data.
6. **From gallery-first discovery to creator-controlled discovery:** artists control what can be shown, indexed, licensed and trained on.
7. **From art as speculation to creative/cultural infrastructure:** support preservation, participation, collective rights and sustainable creator economics.

## 6. First pilot

Start with 100–500 objects only after participating institutions and artists approve the scope. Include multiple media, at least one sculpture/installation, varied provenance completeness, and distinct rights profiles. Use synthetic fixtures for security and conformance; never use synthetic records as empirical impact evidence.

Pilot sequence:
1. baseline current documentation and diligence workflow;
2. register identities and evidence without changing ownership;
3. reconcile provenance and rights with issuers;
4. attach spatial records where authorized;
5. collect exposure/condition data with conservator review;
6. run discovery and discrepancy triage in shadow mode;
7. compare time, completeness, errors and user outcomes;
8. publish limitations and negative results before expanding.

## 7. Stop/go rules

Stop or narrow the pilot if:
- rights or consent are unclear;
- an issuer cannot substantiate the claim scope;
- cultural authority is contested;
- private location/owner data can leak through graph traversal or embeddings;
- geometry is visually convincing but not metrologically validated;
- valuation is miscalibrated or uses non-representative training data;
- a tool silently converts missing provenance into positive authentication.

Proceed to scale only after independent review of schema mappings, security/privacy, rights, benchmark splits and human workflows.

## 8. Selected sources

- [Art Basel & UBS Global Art Market Report 2026](https://www.artbasel.com/stories/the-art-basel-and-ubs-global-art-market-report-2026)
- [Artsy Art Market Trends 2025: survey on transparency](https://www.artsy.net/article/artsy-editorial-art-market-trends)
- [Artlogic: company and product scale](https://artlogic.net/about)
- [Artlogic Certificate of Authenticity tracking beta (updated May 2026)](https://support.artlogic.net/hc/en-gb/articles/360020507139-Certificate-of-authenticity-tracking-beta)
- [ArtCheck provenance verification](https://artcheck.art/en/)
- [Management Science: In Art We Trust](https://pubsonline.informs.org/doi/10.1287/mnsc.2022.4633)
- [South African fine-art pricing ML comparison (2025)](https://www.sciencedirect.com/science/article/abs/pii/S0957417425000909)
- [South African fine-art quantile hedonic study (2009–2021)](https://www.sciencedirect.com/science/article/pii/S026499932300113X)
- [The Met Collection API / Open Access](https://metmuseum.github.io/)
- [ArtBench-10 research](https://www.nature.com/articles/s41598-025-18420-0)
- [SHREC 2026 high-frequency heritage geometry benchmark](https://www.sciencedirect.com/science/article/pii/S0097849326001974)
- [ICOM Object ID](https://icom.museum/en/resources/standards-guidelines/objectid/)
- [CIDOC CRM](https://cidoc-crm.org/)
- [W3C Verifiable Credentials Data Model 2.0](https://www.w3.org/TR/vc-data-model-2.0/)
