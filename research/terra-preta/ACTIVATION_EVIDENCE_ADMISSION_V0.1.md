# Observation-backed hydraulic activation evidence audit v0.1

**Status:** synthetic known-truth evidence-admission evaluator. It is not a field activation classifier, hydraulic solver or CT image-analysis system.

## Why this layer is needed

A pore graph can be topologically correct while its active/inactive edge labels are scientifically unjustified. The graph audit in #6572 therefore needs an evidence-admission boundary:

- CT and segmentation can establish observed morphology under a stated resolution, field of view and support. They cannot alone establish hydraulic activation.
- Tracer or hydraulic observations can support event-specific activation only when their spatial and temporal support maps to the registered graph edge and their calibration/QC is acceptable.
- Sensor-derived estimates and model predictions remain derived evidence, not direct observations.
- Tracer non-detection can support inactivity only when conservative tracer behavior is independently qualified with a hashed protocol/evidence artifact. Reactive/sorptive dyes or unknown tracer behavior cannot convert non-detection into proof of hydraulic inactivity.
- A non-detection does not establish inactivity when sampling is too sparse, does not cover the event, or cannot exclude flow above a predeclared relevant threshold.
- Conflicting qualified direct evidence yields abstention rather than a forced vote or silent averaging.

The data lineage and modality-specific observation-operator details remain owned by #6498 and #5643. This small audit applies admission rules to their evidence references; it does not create a new observation database or model.

## v0.1 admission rules

For a direct observation to support an edge state, the record must have an immutable artifact URI/SHA-256, method/version, calibration/reference identity, passed QC, matching specimen/event, a named spatial-support identity, a registered 3D edge mapping whose support ID matches the graph manifest, with transformation identity, quantified registration error below the task-profile limit, and sufficient spatial-support overlap. A bulk outlet breakthrough curve is a boundary-integrated network response: it cannot identify one edge in a parallel network, even if it is a valid direct tracer observation at the core boundary.

A positive observation may support active state only when the lower confidence bound on measured flux exceeds the event's predeclared minimum relevant flux at or above the task profile's required confidence level. A point estimate above threshold is not enough.

An inactive state requires all of:
- direct tracer or hydraulic observation;
- observation interval covers the complete event;
- sampling cadence is no coarser than the declared event maximum;
- event coverage meets the declared minimum;
- the upper confidence bound on undetected flux is no greater than the minimum relevant flux and meets the required confidence level;
- no conflicting qualified positive observation exists.

An image-only, derived-prediction or model-prediction record cannot confirm hydraulic activity or inactivity. A support mismatch, event mismatch, QC failure, incomplete evidence or conflict produces unassessed.

The event profile declares separate thresholds for tracer mass flux (mg/h) and water-volume flux (mL/h). These quantities are never compared numerically across units; each observation must match its modality's measurand/unit and be judged against the corresponding threshold. The event threshold and coverage rules are task-profile values in the synthetic fixture, not universal soil-physics thresholds. A real profile must pre-register thresholds based on its scientific question, measurement sensitivity and uncertainty.

## Known-truth fixture

Twelve edges cover:
1. event-matched positive tracer evidence;
2. CT geometry without hydraulic evidence;
3. full-window non-detection with sufficient cadence/coverage/bound;
4. sparse non-detection;
5. model-only prediction;
6. conflicting direct positive and negative measurements;
7. evidence from the wrong event;
8. a core aggregate not registered to the specific 3D edge;
9. a boundary-integrated outlet tracer response that cannot identify a single edge;
10. a high-registration-error observation despite a nominal edge mapping;
11. a positive tracer point estimate without a lower confidence bound;
12. a fully sampled negative result from a reactive/sorptive tracer without conservative-behavior qualification.

Expected output: one confirmed-active, one confirmed-inactive and ten unassessed. The conflict control uses a direct water-volume observation and a direct tracer-mass observation; their numerical values are not compared, only whether each independently crosses its own predeclared threshold. All fixture artifacts and values are manufactured.

## Reproducibility and safety

The Rust CLI hashes exact input bytes, validates IDs/references and artifact digests, sorts stable results by input order, writes only to a new path and records a synthetic-only claim ceiling. The focused workflow checks formatting, 18 Rust tests, strict Clippy, Draft 2020-12 schemas, expected admission outcomes and no-overwrite. Unit tests explicitly check both sides of the tracer asymmetry: a positive, edge-resolved sorptive-tracer detection may confirm activity, while a sorptive-tracer non-detection cannot confirm inactivity; a direct water-flux non-detection can qualify independently of tracer chemistry.

## Integration

- #6572 consumes only graph states admitted by this layer.
- #6568 remains the simplified conductance/tracer known-truth operator.
- #5643 owns finite-support CT and tracer observation operators and cross-modal dependence.
- #6498 owns raw sensor observation versus derived estimate lineage.
- #5548 owns geometry extraction and segmentation-threshold sensitivity.
- Symthaea #11446 may later select measurements that resolve a consequential unassessed edge.
- Symtropy #4487 may later escalate physical solvers, but must not treat graph predictions or image-only evidence as direct observations.

## Proposed qualification targets (not achieved)

- Zero geometry-only or model-only promotions to confirmed hydraulic states.
- Zero inactivity claims from inadequate negative sampling.
- 100% manufactured-truth agreement for positive, qualified-negative, sparse, censored, conflicting, wrong-event and mismatched-support controls.
- Deterministic replay, stable digests and fail-closed invalid inputs.
- Empirical calibration only after independent specimen/profile/event holdouts, with false activation, false inactivity, abstention and detection coverage reported separately.

## Claim ceiling

All v0.1 fixture values are synthetic. The evaluator tests evidence semantics only. It does not prove a real pore carried water, estimate flow, identify root origin, predict breakthrough, validate a model or authorize field actions.
