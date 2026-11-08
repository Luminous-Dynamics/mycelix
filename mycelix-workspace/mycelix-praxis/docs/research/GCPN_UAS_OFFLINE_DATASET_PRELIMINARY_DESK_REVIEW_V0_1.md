# GCPN Offline UAS Dataset Qualification — preliminary desk review v0.1

**Review date:** 8 November 2026  
**Review type:** AI-assisted research-source, rights-gate and statistical-design desk review  
**Parent:** [GCPN comparison #5940](https://github.com/Luminous-Dynamics/mycelix/issues/5940)  
**Child:** [Offline dataset qualification #5960](https://github.com/Luminous-Dynamics/mycelix/issues/5960)  
**Disposition:** continue source/metadata desk research; do not download, train on, derive or redistribute candidate data until exact file-level rights and privacy decisions are recorded.

This is not a legal opinion, licence determination, ethics approval, independent statistical review, RF engineering opinion or professional sign-off. No external reviewer has signed this document.

## 1. Exact material reviewed

| Artifact | Branch blob SHA |
|---|---|
| Offline dataset evaluation protocol | `55664cbc071c56bafe338404eb2967c7fbeda3dd` |
| Candidate dataset register | `260df7279598da21f83eeca5fda3108bb3eecaf0` |
| Rust verifier | Added in this branch; exact source digest and CI head are recorded in the PR once qualified |

The register contains three candidate sources. All are explicitly `NOT_ADMITTED_PENDING_...`. None has been downloaded or consumed by this programme. Exact file-level licences, data dictionaries, metadata groupings and file hashes remain unverified. The Rust verifier is intended to enforce this state, not to grant licence or data access.

## 2. Disposition

**Suitable for source/rights desk research and implementation of a source-register verifier. Not yet suitable for dataset download, training, feature extraction, derivative creation, redistribution or comparative performance claims.**

The strongest research choice is to add a separate offline dataset track (B0) between Track A workflow replay and future Track B physical testing. B0 can test modality-specific generalization and dataset leakage without operating equipment or aviation/radio systems. It must not be merged with Track A or treated as evidence of live physical sensor performance.

The three candidate datasets differ in modality, labels, capture conditions, target diversity, grouping metadata and rights. They cannot be pooled into a single leaderboard without explicit semantic mapping and rights review.

## 3. Strengths observed

1. **Non-admission is explicit.** All candidates have a not-admitted status, and the verifier rejects a promotion to admitted status.
2. **Paper licence is separated from file-level rights.** The register does not assume that an open-access article grants every linked data/code artifact the same rights.
3. **DRFF-R2 receives special treatment.** Its paper states CC BY-NC-ND 4.0; derivative sharing remains false and repository-level terms unresolved.
4. **Leakage risk is central.** The protocol prohibits random-window split claims when repeated samples, units, sessions, days or receivers may cross partitions.
5. **Modality limitations are explicit.** TSMS distinguishes raw I/Q from processed range-Doppler frames; the 2.4 GHz set is limited to one aircraft model/band and semi-controlled coexistence conditions.
6. **Claim ceiling is narrow.** Source/split integrity is not represented as detection performance, legal permission or field safety.
7. **No live operations are needed.** The first research stage is offline only and explicitly excludes payload decoding, operator identity, tactical trajectory inference, active countermeasures and field radio monitoring.

## 4. Findings and required closure

| ID | Severity | Finding | Required closure |
|---|---|---|---|
| UDS-PR-01 | Blocker for use | Exact file-level licence for each linked data file, annotation, code sample and metadata artifact has not been verified. TSMS separately states CC BY 4.0 for the dataset in its Data availability section and CC BY-NC-ND 4.0 for article text/images in Rights and permissions; those are distinct objects, not interchangeable licences. | Obtain the dataset repository's current terms and file-level licence, identify rights holders and third-party components, preserve source snapshots with retrieval date, and obtain a written legal/steward determination for intended research and derivative use. Do not copy article figures/text under the dataset licence. |
| UDS-PR-02 | Blocker for DRFF-R2 derivatives | Paper states CC BY-NC-ND 4.0; non-commercial/no-derivatives limitations may apply to paper material and potentially linked artifacts, depending on repository rights. | Do not redistribute adapted data, transformed spectrograms, derived datasets or model artifacts until legal counsel confirms the exact licence, scope and whether the proposed use is an adaptation. |
| UDS-PR-03 | Blocker for raw I/Q | TSMS and RF datasets include raw I/Q. The permitted purpose and whether the capture can contain communications-bearing/private content have not been independently reviewed. | Data privacy/communications reviewer and dataset steward define allowed fields/transforms and prohibit demodulation, private payload decoding or retention. If permissible processing cannot be established, do not consume raw I/Q; use only lawfully supplied labels/features. |
| UDS-PR-04 | Major | TSMS has repeated measurements in a controlled distance grid; exact unit/session/day/receiver grouping needs file-level inspection. | Build a metadata inventory, bind each measurement to capture group and hold out whole groups. If group metadata is absent, mark relevant generalization unsupported. |
| UDS-PR-05 | Major | 2.4 GHz RF coexistence supplied training/validation folders may contain source-group leakage. | Audit source IDs, capture session/day, receiver, hardware and repeated windows before using the validation set. Rebuild grouped splits from raw metadata only if rights permit. |
| UDS-PR-06 | Major | DRFF-R2's reported 26 units/eight models and varied acquisition conditions are paper-level descriptions until exact files/metadata are inspected. | Verify the exact repository version, per-file inventory and group identifiers. Do not assume all seven subsets share identical labels or protocol. |
| UDS-PR-07 | Major | Labels across datasets may not have equivalent meaning (drone signal vs RF coexistence classes vs non-drone target). | Define a typed label mapping with source-level semantics and unsupported/unmappable classes preserved; do not force a universal binary class without a defensible mapping. |
| UDS-PR-08 | Major | Random spectrogram/window splitting can leak repeated signals, capture sessions, physical aircraft, receiver settings or day context. | Preregister session/day/unit/model/configuration holdouts where available; publish split manifest and overlap audit. |
| UDS-PR-09 | Major | Small numbers of physical units/models can make nominal sample counts misleading. | Report independent group counts and group-wise uncertainty; avoid treating windows or repeated measurements as independent observations. |
| UDS-PR-10 | Major | Cross-dataset generalization may be impossible because modality, frequency, acquisition hardware, target labels and preprocessing differ. | Keep per-dataset/per-modality results primary; cross-dataset transfer is supported only after explicit label and feature semantics are qualified. |
| UDS-PR-11 | Major | The published split/metric results, if any, are not independent baselines until exact code, versions, data splits and compute budgets are reproduced. | Pin a baseline implementation, source hashes, environment and resource budget. Compare only like-for-like tasks and denominators. |
| UDS-PR-12 | Observation | A model's ability to classify a drone-signal label does not imply detection of an autonomous, RF-silent aircraft or intent. | Keep label meaning narrow; never infer absence from no RF label and never infer intent/hostility. |
| UDS-PR-13 | Observation | Dataset publication date and open paper availability do not ensure current rights or ongoing hosting/access. | Revalidate the landing page, repository version and rights at retrieval time; record superseded or removed files. |
| UDS-PR-14 | Observation | Public redistribution of even aggregate derived artifacts may be constrained by source licence, privacy or contractual terms. | Rights reviewer classifies every planned artifact: metrics, embeddings, weights, sample visualizations, transformed files and source excerpts. |

## 4A. TSMS article-versus-dataset rights clarification

The source review now records two separate licence statements:
- the article's **Data availability** section says the TSMS dataset is publicly accessible under CC BY 4.0;
- the article's **Rights and permissions** section says the article itself is under CC BY-NC-ND 4.0.

The Figshare+/OpenAIRE landing metadata retrieved for this desk review identifies the dataset but does not expose a file-level licence field. Therefore the register records the article's dataset statement and article licence separately, while leaving exact dataset repository/file terms unverified. This is not evidence that the dataset is necessarily NC-ND, nor permission to reuse article figures, code or third-party assets under CC BY 4.0. The dataset remains not admitted until exact repository/file rights are confirmed.

Sources:
- Article and its separate Data availability / Rights and permissions sections: https://www.nature.com/articles/s41597-026-06802-6
- Dataset landing record: https://explore.openaire.eu/search/dataset?pid=10.25452%2Ffigshare.plus.30027313.v1

## 5. Dataset-by-dataset assessment

### TSMS-Drone
Good fit for studying time-synchronized modality relationships and controlled multi-sensor fusion. Its target diversity is narrow and controlled distances are not a field distribution. Raw I/Q processing needs explicit rights/privacy review. Processed FMCW range-Doppler data must not be called raw FMCW. Keep results limited to the exact named dataset and held-out groups.

### 2.4 GHz coexistence
Good fit for benign RF background discrimination under the documented Wi-Fi/Bluetooth coexistence condition. One drone platform and one band restrict generalization. First priority is an audit of source grouping and the supplied training/validation split. The dataset-specific Mendeley licence must be checked; CC BY on the article is not enough.

### DRFF-R2
Potentially valuable for model/unit/day/configuration diversity. Exact dataset repository terms remain unresolved. CC BY-NC-ND paper licence is a hard legal-review trigger: no derivative publication, transformed artifact redistribution or commercial use assumption. A non-commercial research run is still not automatically cleared for every data use or derived artifact.

## 6. Required independent reviewers

| Reviewer | Required scope |
|---|---|
| Dataset steward / rights counsel | Exact file-level licence, third-party materials, permitted use, adaptation and redistribution scope |
| Privacy / communications-content specialist | Raw I/Q purpose, content minimization, private-content exclusions and retention |
| Independent statistician / ML evaluation reviewer | Group split integrity, repeated-measure independence, confidence intervals, cross-dataset leakage |
| RF/sensing subject-matter reviewer | Measurement semantics, modality limitations, calibration/provenance and label meaning |
| Independent reproducer | Baseline reproduction and evaluator/report verification |

Each review records exact file/DOI/version, retrieval date, hashes, legal scope, methods, exclusions, findings, disposition and reviewer details. No external sign-off has yet been received.

## 7. Qualification stages

- **G0 — register contract:** current. Three candidates listed, none admitted.
- **G1 — rights/privacy desk review:** file-level licence, data rights, raw I/Q and derivative restrictions resolved.
- **G2 — data inventory:** authorized download only; exact file hashes, data dictionary, missingness, capture groups and provenance.
- **G3 — split qualification:** group holdout and overlap audit; unsupported splits explicitly blocked.
- **G4 — offline baseline reproduction:** documented per-dataset baselines with exact code/data/compute revisions.
- **G5 — independent evaluation:** held-out groups, uncertainty, calibration/abstention, worst-group results and cross-dataset limits.
- **G6 — external claims review:** rights, statistical and RF reviewers approve the exact claim scope before publication.

No gate automatically authorizes physical sensors or UAS testing. Track B1 remains separate.

## 8. Final disposition and non-claims

**Continue source/rights desk research and strict register verification. Do not download or run candidate datasets yet.**

This review does not determine the licence of any dataset file, establish legal permission, verify raw I/Q privacy, prove split independence, reproduce a baseline, establish drone detection performance or validate real-world protection. No candidate dataset is currently admitted.
