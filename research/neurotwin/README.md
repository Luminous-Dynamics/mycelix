# NeuroTwin — Mycelix-native MDD research programme

**Status:** research protocol and governance design; no controlled cohort access, model training, clinical validation, or deployment is claimed.  
**Canonical home:** [Luminous-Dynamics/mycelix](https://github.com/Luminous-Dynamics/mycelix).  
**Tracking:** [NEUROTWIN-001 / #6276](https://github.com/Luminous-Dynamics/mycelix/issues/6276).

NeuroTwin investigates whether longitudinal, multimodal and network-aware models can improve reproducible understanding of major depressive disorder (MDD) treatment outcomes. It is a research programme, not a diagnostic product or treatment recommender.

## Programme documents

- [MDD cross-trial benchmark protocol v0.2](MDD_CROSS_TRIAL_BENCHMARK_PROTOCOL_V0_2.md) — published baselines, trial directions, model ladder, prospective translation criteria and benchmark targets.
- [Clinical evidence landscape and SOTA comparison v0.1](CLINICAL_EVIDENCE_AND_SOTA_COMPARISON_V0_1.md) — current comparator results, TRIPOD+AI / PROBAST+AI standards, and a staged benchmark ladder.
- [JAMA cross-trial reproduction checklist v0.1](JAMA_CROSS_TRIAL_REPRODUCTION_CHECKLIST_V0_1.md) — exact cohort overlap, outcome timing, published AUC/BA table, preprocessing and reproduction controls.
- [Data-access and partnership readiness v0.1](DATA_ACCESS_AND_PARTNERSHIP_READINESS_V0_1.md) — current Brain-CODE, NIMH NDA and Stanford BRAINnet access requirements; independent-trial candidates and institutional partnership plan.
- [Mycelix governance and integration contract v0.1](MYCELIX_GOVERNANCE_INTEGRATION_V0_1.md) — how the work composes Personal, Identity, Health, DeSci and federated-learning boundaries without creating parallel patient or consent authorities.
- [Aggregate evaluation receipt schema v0.1](schemas/evaluation-receipt-v0.1.schema.json) — versioned, aggregate-only evidence for a research run.
- [Synthetic receipt fixture](fixtures/synthetic-evaluation-receipt-v0.1.json) — deliberately synthetic contract example; not a study result.

## Central scientific distinction

A model that predicts response under a treatment is **prognostic**. A model that identifies which treatment works better for a particular person must estimate **treatment-effect heterogeneity** and demonstrate value for a treatment policy. Retrospective AUROC is not proof of treatment-selection utility.

The prospective 2026 SMART-D study is a required counterexample in our benchmark reading: reported retrospective biomarker AUCs were 0.66–0.86, yet the preregistered biomarker-consistent versus inconsistent assignment comparison was not significant. NeuroTwin therefore treats prospective policy value, calibration, uncertainty and transportability as core goals—not optional polish.

## Non-negotiable boundaries

- MDD is the only v0.1 clinical domain. Anxiety, epilepsy, stroke and other conditions require independent protocols.
- No participant-level health data, identifiers, images, EEG, embeddings, gradients or individual predictions in public DHT, IPFS, repository fixtures, logs or public evaluation receipts.
- Controlled data remain in the data provider's approved environment and within the approved study scope.
- Mycelix identity, a signed event, a consent card, a data hash or a model artifact does not by itself establish clinical authority, lawful use, source truth or current consent enforcement.
- Existing Personal Health Vault privacy, consent lifecycle and caller-bound disclosure work are hard dependencies. NeuroTwin must not bypass them.
- Symthaea/HDC/CfC/LTC methods are candidate research model families; they are not presumed biologically valid or clinically superior.
- No diagnosis, prescribing, medication change, crisis triage or autonomous clinical action.

## Current state

| Capability | State |
|---|---|
| Published cross-trial benchmark identified | Literature reviewed; source reproduction not yet run |
| 2026 prospective treatment-guidance comparator identified | Literature reviewed; not a Mycelix validation |
| Controlled EMBARC / CAN-BIND data access | Not authorized in this programme |
| Mycelix research governance contract | Design proposal |
| Aggregate receipt schema | Protocol artifact; not yet enforced by a service |
| Clinical validation / prospective deployment | Not performed |

**Do not upgrade any row based on documentation, a queued workflow, synthetic fixture, or model-generated result.**
