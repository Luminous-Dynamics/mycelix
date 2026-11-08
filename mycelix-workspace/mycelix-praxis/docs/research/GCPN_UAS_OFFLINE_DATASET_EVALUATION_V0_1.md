# GCPN — Offline public-dataset evaluation for civilian UAS awareness v0.1

**Parent:** [Matched awareness comparison #5940](https://github.com/Luminous-Dynamics/mycelix/issues/5940)  
**Child:** [Offline public-dataset qualification #5960](https://github.com/Luminous-Dynamics/mycelix/issues/5960)  
**Status:** source candidates and qualification protocol only; no dataset is yet admitted for execution  
**Claim ceiling:** source-register and split-protocol integrity only

## 1. Why add an offline dataset lane?

There are now useful 2026 public datasets for drone-awareness research. They make it possible to study sensor-domain limitations and fusion offline, without importing equipment, operating radio apparatus or flying aircraft in South Africa.

Keep three evaluation tracks separate:

- **Track A — event-envelope replay:** downstream evidence and human-review workflow using identical event envelopes.
- **Track B0 — offline public-dataset analysis:** model/feature evaluation on exact, rights-cleared published data and leak-audited splits.
- **Track B1 — physical sensor trial:** live equipment and independently adjudicated ground truth; still blocked by site, aviation, radio, vendor, privacy, insurance and professional approval.

An offline dataset result is not a physical detection test, range guarantee, field performance claim or approval to operate equipment.

## 2. Candidate source review

### TSMS-Drone — synchronized CW/FMCW/RF

- Paper: Han & Jung, *A Time-Synchronized Multi-Sensor drone dataset acquired from multiple radars and RF receiver*, Scientific Data (10 February 2026), https://doi.org/10.1038/s41597-026-06802-6.
- Dataset DOI: https://doi.org/10.25452/figshare.plus.30027313.v1.
- Dataset landing reference: https://explore.openaire.eu/search/dataset?pid=10.25452%2Ffigshare.plus.30027313.v1.
- Published contents: synchronized continuous-wave radar, FMCW radar and RF receiver; four commercial drone models plus one non-drone target; controlled distance conditions 2–30 m in 2 m increments; 500 repeated measurements per condition.
- Data format: raw I/Q for CW and RF; processed FMCW range-Doppler frames, not full raw FMCW ADC streams; matching images and sample MATLAB code.
- Rights note: the paper says the dataset is available under CC BY 4.0. Before use, verify the exact dataset landing-page licence, any third-party component notices and the licence on every code/data artifact. Do not assume the paper licence automatically controls every linked artifact.
- Important limitation: controlled, repeated measurements with a very small target set do not provide representative field performance or broad aircraft diversity.

### 2.4 GHz RF coexistence dataset

- Paper: *A labeled RF signal dataset for UAV detection and classification in the 2.4GHz band under Wi-Fi and Bluetooth coexistence*, Data in Brief (August 2026), https://doi.org/10.1016/j.dib.2026.113127.
- Dataset DOI: https://doi.org/10.17632/y4w96kzsb5.1.
- Published contents: 6,842 spectrograms and 22,375 labelled signal instances; Drone_Signal, Wi-Fi and Bluetooth classes; one DJI Phantom 4 Pro platform; indoor/semi-controlled data acquisition with simultaneous Wi-Fi/Bluetooth activity. The described acquisition uses a 2.438 GHz centre frequency and discusses 30/40/56 MHz sampling bandwidths.
- Published split: 5,678 training and 1,164 validation images. Do not assume these form a leakage-resistant independent split without verifying capture-session, day, device, and source-group identities.
- Rights note: the article is open access under CC BY, but the Mendeley dataset-specific licence and rights for every file/annotation must be inspected separately.
- Important limitation: one aircraft platform, one band and limited RF environment. The authors themselves describe generalization to other UAV models, bands and protocols as unvalidated.

### DRFF-R2 — multi-scenario RF dataset

- Paper: Zheng et al., *A Multi-Scenario UAV RF Dataset with Real-World Acquisition and Signal Processing Benchmarking*, Scientific Data (22 October 2026), https://doi.org/10.1038/s41597-026-08490-8.
- Public preprint: https://arxiv.org/abs/2603.00106.
- Published description: seven subsets; recordings from 26 UAV units across eight models; varying flight states, altitudes, speeds, acquisition days and receiver configurations.
- Rights note: the article states CC BY-NC-ND 4.0. Under that paper licence, adaptations of the article material are not permitted and use is non-commercial. The dataset repository's own licence must be checked separately. Do not publish transformed copies, derivative datasets, derived spectrograms or model artifacts without written rights clearance.
- Important limitation: a research dataset does not itself prove that any RF modality will detect non-transmitting/autonomous aircraft or generalize to a new site.

The machine-readable candidate register records the source snapshots and outstanding qualification questions. None of these datasets is admitted for model development until the exact data repository, licence, data dictionary, missingness, provenance and file hashes have been reviewed.

## 3. Permitted scientific question

Evaluate only passive, coarse event-presence awareness or separation of dataset-provided signal classes where the public labels support it.

Permitted:
- dataset-specific detection / non-detection classification;
- benign RF coexistence discrimination (e.g., published Wi-Fi and Bluetooth classes);
- modality robustness and missing-modality evaluation;
- calibration, abstention and domain-shift analysis;
- error analysis by dataset, acquisition group and documented context.

Excluded:
- operator/controller identification or geolocation;
- individual aircraft serial/RF fingerprint identification;
- demodulation, decoding or retention of private communications payloads;
- inference of precise flight paths, altitude, speed, bearing or tactical trajectories;
- command, takeover, jamming, spoofing, interception or engagement;
- deployment or live RF monitoring under this offline issue.

The TSMS and RF-coexistence materials include RF receiver data. Use only the documented feature/label information needed for passive presence awareness. Do not attempt to decode communications content.

## 4. Leakage-resistant split protocol

Random spectrogram/window splits are not acceptable as evidence of out-of-sample generalization where repeated samples, the same aircraft, receiver, capture session, day or environment occur in both training and evaluation.

Require grouping metadata first, then freeze one or more held-out protocols:
- **session holdout:** no capture session appears in more than one partition;
- **day holdout:** where day metadata exists, hold out entire acquisition days;
- **unit holdout:** hold out physical UAV units, not just repeated windows;
- **model holdout:** for datasets with enough models, hold out an entire model family;
- **receiver/configuration holdout:** hold out receiver/configuration combinations where supported;
- **cross-dataset:** train on one source and evaluate on another only where labels and input semantics are truly comparable.

If the source metadata cannot support a claimed group split, label that generalization question **UNSUPPORTED**, not passed. The RF coexistence dataset's supplied training/validation folders must be audited for group leakage before being treated as a held-out validation set.

Do not pool incompatible classes or modalities into a single leaderboard. Report per-dataset and per-modality results first; cross-dataset results are separate and require explicit class mapping.

## 5. Metrics

- class-wise precision, recall, F1 and PR-AUC with exact denominators;
- false positives per labelled negative sample, or per recorded negative-time unit only where valid duration metadata exists;
- errors under Wi-Fi/Bluetooth/background coexistence strata;
- Brier score and reliability only with adequate held-out sample counts;
- coverage versus selective error and explicit abstention;
- worst-group performance and uncertainty intervals;
- cross-dataset generalization gap;
- missing/corrupt-modality handling;
- resource/runtime cost on a fixed, declared compute budget;
- data rights, source provenance, preprocessing and split integrity.

No universal superiority target is set until each dataset and its reference baselines have been independently qualified.

## 6. Reproducibility and data rights

For every admitted source, pin:
- exact landing page, DOI, access date and file-version identifier;
- file-by-file SHA-256;
- dataset/data-dictionary version and annotation provenance;
- exact file-level licence and permitted-purpose determination;
- privacy/content review for raw I/Q;
- allowed transforms and exact transform code/version;
- split manifest and group identifiers (appropriately minimized);
- evaluator commit, environment, compute hardware and output receipt.

Do not commit licensed source data, raw I/Q or large derived artifacts into Mycelix. Store only allowed manifests, exact digests, split IDs at the minimum necessary granularity, code, aggregate results and audit receipts. For NC-ND materials, obtain specific legal advice before distributing anything that could be considered an adaptation or derivative.

## 7. Professional review gates

- dataset/data-rights counsel or steward for each repository and each file class;
- privacy / communications-content reviewer for raw I/Q material;
- independent statistician/data-science reviewer for group splits and leakage audits;
- RF/sensing subject-matter reviewer for modality limits and label meaning;
- independent evaluator for reproduction before comparative claims.

## 8. Claim ceiling

A passing manifest/split verifier establishes source-register and split-contract integrity only. A later offline result establishes measured performance on the exact named dataset, version and split. Neither establishes physical-site performance, broad real-world detection, intent, airspace permission, legal authorization, or protection from attack.
