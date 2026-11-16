# NeuroTwin clinical digital-twin evidence ladder v0.1

**Status:** research architecture and qualification proposal. This is not a claim that NeuroTwin is already a validated digital twin, physiological simulator or clinical decision tool.  
**Canonical home:** `Luminous-Dynamics/mycelix`.  
**Programme:** [NEUROTWIN-001 / #6276](https://github.com/Luminous-Dynamics/mycelix/issues/6276).

## 1. First principle: a digital twin is a claim about correspondence

A statistical model trained on people with MDD is not necessarily a twin of any individual. A brain-connectome simulator is not necessarily a model of the person's current clinical state. A model that predicts an outcome after observed treatment is not necessarily able to predict what would have happened under a different intervention.

NeuroTwin should use the term **clinical digital twin** only when a model has an explicit, measured correspondence to an individual or a defined individual-state trajectory, can update from that individual's observations, expresses uncertainty and missingness, and has been validated for the exact prediction or intervention claims being made.

The person is not reducible to their brain scan, connectome, diagnosis, medication history or model state. Any longitudinal representation is a partial, purpose-bound model of observations—not an ontological representation of the person or a replacement for their voice, agency or care relationship.

## 2. Evidence tiers

| Tier | Name | Minimum evidence | Allowed claim | Forbidden inference |
|---|---|---|---|---|
| 0 | Population predictor | Retrospective cohort model with defined endpoint and external evaluation | Predictive association for a declared population/task | Not an individual's twin; no counterfactual intervention claim |
| 1 | Longitudinal individual-state estimator | Repeated observations; explicit time index; missingness model; update rule; prospective within-person state tracking; calibrated uncertainty | Estimates a limited set of observed/latent state variables for a declared individual and time horizon | Not necessarily mechanistic; no claim that state updates reveal causal treatment effects |
| 2 | Dynamical digital surrogate | Declared state variables and units; transition/observation equations; parameter identifiability; observability analysis; numerical verification; calibrated multi-step forecasts | Simulates the declared state trajectory within validated support | A numerical simulation is not biological truth; no new intervention claim without intervention validation |
| 3 | Intervention-qualified causal twin | Explicit target-trial estimand; well-defined treatment/intervention; causal assumptions; identification strategy; positivity; measured confounding and sensitivity analysis; randomized or otherwise credible intervention evidence | Estimates a bounded counterfactual or treatment-effect quantity for the declared population and intervention | No individualized treatment recommendation until policy value is independently evaluated |
| 4 | Prospective silent twin | Frozen model, state updater and intervention semantics; prospectively collected observations; silent predictions; drift, calibration, missingness and abstention monitoring | Prospective predictive performance and operational feasibility under silent evaluation | Predictions do not alter care; silent validation is not treatment benefit |
| 5 | Randomized policy-evaluated twin | Locked intervention policy; adequate randomized impact study; prespecified patient-centred outcome, harms and fairness; independent analysis | Evidence that a specific policy improves specified outcomes in the studied setting | No blanket generalization to other populations, treatments, sites or health systems |
| 6 | Governed clinical operation | Appropriate independent clinical, safety, security, ethics, regulatory, legal and operational review; post-deployment monitoring and rollback | Only the exact authorized clinical function and jurisdiction | No autonomous scope expansion or self-certification |

The tier is claim-specific. A system may be Tier 4 for a narrow forecasting task while remaining Tier 0 for treatment selection. A new modality, new population, new treatment or new intervention semantic can require a separate evidence path.

## 3. What should the twin represent?

The candidate is a **longitudinal, multimodal, partially observed clinical state**, not a static connectome.

Potential state domains, subject to data authority and scientific justification:
- symptom severity and domain-specific symptom trajectories;
- treatment exposure, adherence and intercurrent treatment changes;
- sleep, activity and behavioural observations;
- contextual and functional measures where lawfully collected and validated;
- brain-network and task-related neurophysiological features when the relevant acquisition exists and meets quality rules;
- patient-reported goals and functional outcomes where appropriate for the study.

Do not treat unavailable modalities as zeros or generate synthetic substitutes. An individual state estimate must explicitly represent what was measured, when it was measured, provenance, measurement uncertainty and missingness.

Do not infer unobserved psychological constructs, personal preferences, identity or causal mechanisms merely because a latent embedding correlates with an endpoint. Latent states need an interpretation contract, identifiability analysis and empirical validation.

## 4. Dynamical model contract

For a declared state (x_t), observations (y_t), intervention (a_t), parameters (	heta) and exogenous/context variables (c_t), a candidate twin may use:

[
x_{t+1}=f_	heta(x_t,a_t,c_t,Delta t)+w_t
]

[
y_t=g_	heta(x_t,m_t)+v_t
]

where (m_t) records the observation modality and missingness state; (w_t) and (v_t) are process and measurement uncertainty. This notation is a model contract, not a claim that the true clinical system is Markovian or fully observed.

Every admitted dynamical model must declare:
- state variables, units, ranges, transformations and measurement mappings;
- observation times, sampling irregularity and (Delta t) semantics;
- intervention variables and their real-world operational counterparts;
- constraints, boundary conditions and conservation/monotonicity assumptions where justified;
- model structure, parameter priors/estimates and identifiability;
- observability and state reconstruction assumptions;
- solver, step size, tolerances, stability and numerical verification;
- process/measurement uncertainty and calibration horizon;
- model failure conditions and abstention support.

A graph-edge edit, neural activation, HDC vector displacement or latent-space perturbation is not a real treatment unless there is a validated mapping from that operation to an intervention delivered to the person and evidence that the model response corresponds to observed intervention effects.

## 5. Counterfactual intervention contract

For treatment selection, the relevant target is not merely (P(Ymid X,A)). It is a declared contrast such as:

[
	au(x)=mathbb{E}[Y^{a=1}-Y^{a=0}mid X=x]
]

The outcome, treatment strategies, follow-up, assignment mechanism, population, intercurrent-event strategy and estimand must be specified before model training. Identification must state the assumptions and evidence supporting it.

A valid intervention semantics record must define:
- exactly what differs between the two treatment strategies;
- dose, duration, sequence, adherence and co-intervention handling as relevant;
- the target population and time zero;
- the endpoint and time horizon;
- randomization or confounding-control assumptions;
- positivity/overlap and the support in which the contrast is identified;
- how uncertainty and unmeasured-confounding sensitivity are represented;
- why the model's simulated action corresponds to the real intervention.

Do not label latent perturbations or arbitrary connectome rewiring as a treatment counterfactual.

## 6. Validation should test trajectories, not just endpoints

### State estimation
- Within-person state reconstruction error and uncertainty calibration.
- Temporal alignment, measurement error, missing modality and irregular sampling.
- Leakage controls across visits and overlapping trial stages.
- Comparison against simple longitudinal baselines and clinical-only filters.

### Forecasting
- One-step and multi-step forecast error across declared horizons.
- Error accumulation and calibration as horizon increases.
- Comparison against persistence, linear mixed-effects/state-space and clinical trajectory baselines.
- Site/trial/device/time transport with predeclared abstention.

### Intervention simulation
- Recovery of known intervention effects in synthetic systems before empirical analysis.
- Parameter identifiability and sensitivity.
- Agreement with independent intervention observations, randomized where possible.
- Falsification: interventions that should have no effect, impossible state transitions, wrong-sign effects and out-of-support actions.
- Counterfactual calibration only where a credible identification strategy exists.

### Policy evaluation
- Frozen policy versus best clinical-only policy.
- Randomization-aware policy-value inference at the actual independent unit.
- Patient-centred outcomes, harms and subgroup performance.
- Silent prospective evaluation before any policy can affect care.

## 7. Benchmarks to aim for

These are **proposed engineering/research targets**, not clinical safety standards or evidence already achieved.

| Area | Target to investigate | Qualification condition |
|---|---|---|
| State tracking | Better than persistence and clinical-only state-space baselines by a preregistered, uncertainty-bounded margin | Same observation schedule and held-out people; no future data |
| Multi-step forecast | Performance and calibration degrade gracefully with horizon; no unsupported extrapolation | Report every horizon and baseline, not one averaged score |
| Cross-trial transport | Preserve clinically meaningful calibration and useful discrimination across a third independent trial | No tuning/recalibration on the final test trial |
| Missing modality | Graceful degradation and explicit abstention; no synthetic backfill | Matched missingness patterns and worst-case stress tests |
| Calibration | Calibration intercept/slope, Brier/log loss and uncertainty with intervals by domain | Sample size, class balance and identifiability reported |
| Selective prediction | Risk decreases as unsupported cases are withheld, with a disclosed coverage cost | Evaluate both model error and harms of withholding |
| Counterfactual fidelity | Simulated treatment contrasts agree with independent randomized effects within prespecified uncertainty | Identified estimand, known treatment mechanism and sensitivity analysis |
| Policy value | Positive paired policy-value difference versus clinical-only policy in adequately powered randomized evaluation | Design-aware uncertainty and independent review |
| Human outcome | Improvement in prespecified patient-centred outcome without unacceptable harm or inequity | Prospective randomized study and independent oversight |

Do not set a universal AUC as the digital-twin threshold. The claim must match the task: tracking, forecasting, causal intervention or policy value.

## 8. Symthaea, Mycelix and Symtropy responsibilities

- **Symthaea:** candidate state representations, HDC/CfC/LTC dynamics, multimodal inference, uncertainty and abstention. Experimental representations must compete against simple clinical and statistical baselines on matched data.
- **Symtropy:** numerical/dynamical simulation infrastructure, equation and solver contracts, parameter sensitivity, boundary checks and replayable intervention experiments. It is not a validated physiological simulator by default.
- **Mycelix Personal / Health Vault:** participant-controlled data boundary and selective disclosure, subject to the actual private-entry and caller-bound authorization qualification.
- **Mycelix Identity:** researcher/institution/service identity and verified role credentials; a DID or trust score does not prove licensure, ethics approval or lawful purpose.
- **Mycelix DeSci / provenance:** protocol versions, data source/release references, model/code identity, approvals and disclosure-reviewed aggregate evidence. A hash proves commitment to bytes, not clinical truth.
- **Institutional research partner:** clinical governance, ethics, source approvals, data custodianship and accountability.

No model, agent or subsystem may self-approve a data-use request, certify its own clinical validity or elevate its own evidence tier.

## 9. Ethical and epistemic limits

The digital twin is a fallible and partial model created for a declared purpose. It does not own the person's identity, values, interpretation or decisions. A participant may correct data, limit uses, withhold optional modalities and withdraw from future participation subject to law, study rules and the limits on already disclosed copies/derived artifacts.

The system must say what it knows, what it does not know, what observations are stale or missing, and which requests are outside validated support. An abstention must route to a declared human/research workflow; it must not silently disappear cases or shift risk onto a participant.

The following are not evidence of a valid clinical twin by themselves:
- high internal AUROC;
- an attractive 3D brain visualization;
- a connectome graph;
- a high-dimensional embedding;
- a low reconstruction loss on synthetic data;
- an HDC/CfC/LTC simulation that has not been calibrated to independent observations;
- an immutable receipt or trust score;
- an offline treatment simulation without validated intervention semantics.

## 10. Current maturity

The present NeuroTwin programme remains a research design and evaluation-tooling effort. Controlled EMBARC/CAN-BIND access has not been authorized in this project; no clinical dataset has been downloaded or analysed here. The current tools evaluate synthetic or already-authorized aggregate predictions; they do not create a digital twin of a patient.

The evidence ladder is intended to keep future claims proportional to the validation performed. Advancement requires independent evidence at each tier, not a documentation update.
