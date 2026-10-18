<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->
# Visual Benchmark Evidence Profile

**Status date:** 2026-10-17  
**Tracking issue:** [VIS-EVID-001 #5157](https://github.com/Luminous-Dynamics/mycelix/issues/5157)  
**Parent program:** [Symthaea VIS-TECH-001 #7820](https://github.com/Luminous-Dynamics/symthaea/issues/7820)  
**Status:** Proposed mapping onto existing scientific-evidence semantics; no new canonical event/receipt type is authorized here.

## Purpose

Provide a narrow evidence profile for visual technology research artifacts: software benchmark outputs, display calibration records, optical-model fixtures, projector measurements, AR/VR timing captures, CGH/light-field outputs, manufacturing inspection datasets and (later) human-study aggregates.

The responsibility is catalogue, provenance, rights-aware sharing and replication linkage. Mycelix is not the optics simulator, measurement instrument or scientific truth oracle.

## Ownership / reuse first

- Reuse [SCI-EVID-001 #2410](https://github.com/Luminous-Dynamics/mycelix/issues/2410), which types the artifact itself separately from a claim or interpretation.
- Preserve [SCI-AUTH-001 #2408](https://github.com/Luminous-Dynamics/mycelix/issues/2408): a producer's derived analysis must not take canonical evidence authority.
- Use existing Mycelix-DeSci publication/lineage flows where suitable. Inspect current semantic ownership before adding any visual-specific code.
- Compose with Symthaea's benchmark manifest and Symtropy's scenario/evaluation substrate; do not invent another generic receipt, signatures-only trust system, or consensus protocol for lab data.

## Paradigm shift: evidence is typed and scoped

Do not treat every uploaded file as equal evidence. Separate:
1. **Artifact kind** — what the bytes are (raw sensor samples, camera capture, calibration profile, synthetic fixture, model output, report, source code, study aggregate).
2. **Measurement/provenance context** — how and where they were produced.
3. **Claim relation** — whether a specific claim is supported, refuted, inconclusive or not evaluated.
4. **Authority/review** — whether a reproduction or review exists and what it actually covered.
5. **Rights/privacy** — whether the bytes and derived artifacts may be accessed, replicated or redistributed.

A content hash proves byte identity relative to that digest. A signature proves a signing key attested to a payload. Neither alone proves instrument calibration, correct methodology, a truthful label, physical correctness, independent replication or redistribution rights.

## Required profile fields

The first implementation should map fields to existing canonical types, not create a parallel schema until a gap is documented.

### Identity and lineage

- Stable artifact/case ID and artifact-kind identifier.
- Exact source commit, executable digest, model/configuration digest and toolchain/dependency lock identity when relevant.
- Frozen corpus, oracle and benchmark-version digests.
- Input and output artifact digests with retrievable content locations or explicit retention/access limitation.
- Transformation/processing lineage, including preprocessing, crop/resize, color management, tone mapping, compression and coordinate transform when relevant.
- Producer and independent reviewer/reproducer references, kept distinct for independence claims.

### Device and measurement context

- Device make/model, panel/optics/runtime/firmware version and configuration where publication is allowed.
- Instrument type, serial/identifier when appropriate, calibration record and validity interval.
- Capture/evaluation time, environment (ambient light, temperature, distance, screen geometry) as relevant to the specific metric.
- Units, color space/white point, transfer function, coordinate frame, wavelength, pixel pitch, sampling rate or frame-time convention where applicable. For light-field results also record observation angle and convention, image-depth position, angular sampling/viewpoint density, angle-resolved MTF, crosstalk metric, luminance by angle, exposure time, scan state and synchronization condition.
- Repetition number, sample denominator, uncertainty/confidence interval and known exclusions.
- Missing context must remain missing; do not infer device properties from file names or prose.

### Open display metrology run records

Compose with Symthaea's [ODRH-001 open display hardware issue #8030](https://github.com/Luminous-Dynamics/symthaea/issues/8030), [ODRH-MET-001 decision-capable metrology issue #8049](https://github.com/Luminous-Dynamics/symthaea/issues/8049), the [reproducibility protocol](https://github.com/Luminous-Dynamics/symthaea/blob/research/odrh-001-open-display-hardware-2026-10-18/docs/engineering/DISPLAY_METROLOGY_REPRODUCIBILITY_PROTOCOL.md), and the [machine-readable measurement run template/schema](https://github.com/Luminous-Dynamics/symthaea/blob/research/odrh-001-open-display-hardware-2026-10-18/docs/engineering/display-measurement-run-template-v0.1.json). Mycelix should anchor or authorize the existing record under canonical scientific-evidence semantics; it must not define a competing run schema or turn a ledger receipt into metrology certification.

For an ODRH run, preserve references/digests for:

- stable run ID, record state, exact protocol ID/revision, source commit and configuration;
- device-under-test identity/revision, host/output mode, signal format/range, pattern sequence digest, OSD profile, APL/dwell and declared measurement geometry;
- instrument identity, calibration record/currentness, sample clock/rate and uncertainty-method identity;
- each measurand and unit, value or explicit NOT_EVALUATED state, expanded uncertainty, minimum decision effect (MDE), decision-capability state and uncertainty budget/covariance treatment;
- raw artifact IDs/content digests/access/retention state, invalid samples, exclusions/aborts, operator/procedure changes and independent reviewer/reproducer;
- claim scope and the distinction between instrument/geometry verification, bench measurement, independent review and physical qualification.

The run template is intentionally PLAN_ONLY with four NOT_EVALUATED observations and no instruments/raw artifacts. That is a planning example, not measured evidence. ACQUIRED_UNQUALIFIED must not be promoted merely because the schema validates, a producer writes PASS, an output file exists, or CI succeeds. Unsupported metrics remain NOT_EVALUATED with null value; missing calibration or an uncertainty budget that cannot resolve the declared MDE blocks that metric from candidate ranking. The proposed U95_total <= MDE/3 gate is a project-level capability hypothesis, not an IDMS requirement or universal metrology rule.

For the first open CAD fixture, preserve the exact native-source digest, license, OpenSCAD/tool version, parameter configuration, mesh bounds/triangle count and invalid-parameter tests. A successful geometry export is a source/build check only: physical fit, part tolerance, material strength, optical baffling, instrument calibration and measured display performance remain separate evidence gates. Per-run STL hashes are provenance; byte-level identity must not be assumed across different OpenSCAD builds unless that reproducibility requirement has been demonstrated. The current [CAD check receipt](https://github.com/Luminous-Dynamics/symthaea/blob/research/odrh-001-open-display-hardware-2026-10-18/hardware/open-display-reference/validation/odrh_sensor_plate_cad_check-v0.1.json) explicitly leaves physical validation NOT_PERFORMED.

### Deterministic display stimulus source artifacts

Compose with Symtropy [ODRH-STIM-001 #2046](https://github.com/Luminous-Dynamics/symtropy/issues/2046) and its draft implementation [PR #2049](https://github.com/Luminous-Dynamics/symtropy/pull/2049). It maps raster files to existing Symthaea `VIS-DISP-CORPUS-001` case IDs; it does not create a parallel benchmark corpus or physical measurement record. The initial source recipes cover only four explicitly defined raster primitives: neutral code ramp, near-black code-value bars, explicit-code center patch, and encoded RGB gradient.

For each generated source raster, anchor or reference:

- immutable source corpus ID and pinned source Git blob digest; original case ID and recipe-manifest ID/version;
- raw recipe manifest SHA-256, normalized selected-recipe digest, recipe schema SHA-256 and output-report schema SHA-256;
- exact raster output SHA-256, output-byte count, dimensions, PPM P6 format, channel order, maxval/code depth and 16-bit sample byte order where applicable;
- generator version/source revision and explicit code-domain/transfer semantics;
- `SYNTHETIC_STIMULUS`, `UNQUALIFIED`, and `physical_output_validation = NOT_PERFORMED` claim state.

A raster digest proves the generated bytes only. It does not prove those bytes were preserved by GPU, OS color management, compositor, signal interface or monitor, nor does it establish physical luminance, chromaticity, HDR mode/metadata, motion cadence, power use or human perception. The current PPM artifacts intentionally include no ICC profile or transport metadata. Instrumented display records in ODRH-MET-001 must link a separate acquired run and raw physical observations rather than promote the stimulus report itself to a measurement receipt. Large raster/raw files should remain in ordinary versioned or content-addressed storage; Mycelix should anchor compact identities and access/provenance metadata using the existing evidence semantics.

### Nanoimprint/grating-route evidence

For AR waveguide manufacturing, compose with [Symthaea VIS-MFG-001 #8012](https://github.com/Luminous-Dynamics/symthaea/issues/8012) and preserve distinct IDs for master/template, resist/material lot, tool profile, each replicated optic and final assembly. Where NIL is used, record master revision/use-cycle and inspection, replica pattern digest, feature geometry/fidelity over the declared field, residual-layer thickness/uniformity, overlay/registration, fill/demolding defect map, calibration/inspection coverage and source/tool provenance. Downstream module acceptance must bind the replicated grating to its waveguide/light-engine pair and angle-/wavelength-resolved optical measurements. The process route is still a planning profile until local tool/process capability, raw metrology and final module behavior are evidenced. Literature-reported resolution or a master-template specification does not qualify the replica or module.

### Test-pattern/context robustness

Compose with [Symthaea VIS-METRO-001 #7894](https://github.com/Luminous-Dynamics/symthaea/issues/7894) and its [pattern robustness and uncertainty protocol](https://github.com/Luminous-Dynamics/symthaea/blob/research/vis-tech-001-program-2026-10-17/docs/engineering/DISPLAY_PATTERN_ROBUSTNESS_AND_UNCERTAINTY.md). For optical/temporal measurements, retain exact stimulus bytes or generator parameters and digest, pattern-family/standard identifier and edition, patch geometry/size/APL, color distribution, spatial background, display mode, preceding stimulus and washout, dwell/cadence, ROI and instrument field of view. Record per-pattern results, repetitions, cross-pattern range, predeclared worst stratum, exclusions and uncertainty; never publish only the best pattern or a mean that hides context-dependent behavior.

A pattern-sensitivity envelope is not a confidence interval. When energy intervals are used to compute a conservative percent-saving interval, preserve the interval method, bounds and any dependency/covariance assumption. If the conservative lower bound does not meet the preregistered energy target, the energy gate is inconclusive even when the point estimate reaches the target.

### Route-frontier comparisons and accounting compatibility

When registering a comparison across manufacturing-route candidates, preserve the target-module profile digest, comparison-protocol digest, evidence class, currency code and cost-accounting-boundary identity. Candidate results are incomparable if any of these differ. The metric vector should preserve conservative upper bounds for cost, energy and cycle time, lower bounds for final calibrated-good yield, and the dispositions of optical, reliability, minimum-yield and evidence-completeness gates. A candidate that fails a mandatory floor is not admitted to the Pareto frontier just because it is cheap. Record whether the values are a synthetic scenario, simulation or bench-measured evidence; Pareto arithmetic does not upgrade evidence maturity.

### Route-Pareto candidate evidence identity

For a manufacturing-route Pareto candidate, require an exact target-module acceptance-profile digest, comparison-protocol digest, currency code, accounting-boundary identity, evidence-class label, evidence-bundle digest and uncertainty-method digest. Preserve conservative upper bounds for unit cost, energy and cycle time; lower bound for final calibrated-good yield; and independent dispositions for optical, reliability, minimum-yield and evidence-completeness floors. Candidate pairs with different targets, protocols, currencies, cost scopes or evidence classes are incomparable. Candidate rankings over synthetic scenarios remain synthetic; a digest or Pareto result is not a process-qualification or manufacturing-truth receipt.
 
### Manufacturing cost and inspection metrics

Compose with the isolated [Symthaea display-manufacturing-economics reference](https://github.com/Luminous-Dynamics/symthaea/tree/research/vis-tech-001-program-2026-10-17/tools/display-manufacturing-economics). For a batch summary, bind one cost currency and accounting boundary, batch/lot identity, unique incoming-unit count, first-pass final-good count, final calibrated-good count, and cost ledger categories. Preserve materials, labor, equipment/facility allocation, energy/utilities, inspection/test, rework/retest, packaging/logistics and incremental scrap handling separately to prevent hidden double counting.

Record cost per calibrated-good unit only when that final-good denominator is positive. Zero final-good output is a valid zero-yield result, but unit cost remains undefined; it must not be represented as zero. For inspection, retain eligible and inspected populations, true/false positives and false negatives, reference-review method, and resulting coverage/precision/recall. Undefined denominators stay undefined; the accounting helper does not establish ground-truth label quality or measured economic completeness.

### Display manufacturing route and stage evidence

Compose with [Symthaea VIS-MFG-001 #8012](https://github.com/Luminous-Dynamics/symthaea/issues/8012) and the [UNQUALIFIED four-route/41-stage profile manifest](https://github.com/Luminous-Dynamics/symthaea/blob/research/vis-tech-001-program-2026-10-17/docs/engineering/display-manufacturing-route-profiles-v0.1.json). Do not add a new manufacturing receipt or process-flow ontology. Preserve references to canonical MFG-PROC/SEMI-PROC stage and state-transition contracts.

For a route/stage artifact, bind the exact route profile/version and digest, stage ID and dependency/branch condition, input/output subject/state references, upstream wafer/die/component lot lineage, process-family and equipment-capability profile refs, and the declared state of evidence (plan/model/attempt/observed/accepted/rejected/unknown). Keep intended output constraints separate from post-step measured output. A completed command is not proof of transformation, and an accepted component does not imply a qualified final display.

Where a production run is actually evidenced, preserve per-stage attempted/inspected/accepted/rejected/reworked/scrapped/unknown denominators, inspection coverage and defect taxonomy, calibration/currentness, raw metrology digest/access status, uncertainty, and the disposition/hold/rework branch taken. Never overwrite original defect/failure records after repair or retest. Route-level conditional dependencies must be explicit; a skipped optional stage does not create an inferred output state.

For economics, bind the cost-accounting boundary and numerator inputs (material, energy, inspection, process time, rework/scrap, equipment allocation) together with unique final calibrated-good-unit denominator. Keep first-pass yield, post-repair yield, final calibrated-good yield and reliability fallout separate. A missing stage result or unresolved denominator blocks route-level yield/cost claims; synthetic route profiles remain planning data only.

### Task-conditioned energy and task-outcome evidence

Compose with [Symthaea VIS-ENERGY-001 #7878](https://github.com/Luminous-Dynamics/symthaea/issues/7878) and its [protocol/reference package](https://github.com/Luminous-Dynamics/symthaea/blob/research/vis-tech-001-program-2026-10-17/docs/engineering/VISUAL_UTILITY_PER_JOULE_PROTOCOL.md). Treat the following as evidence context, not a new canonical receipt or scoring authority:

- paired baseline/treatment run IDs, exact device/profile/firmware identity, adaptation policy, declared task window and task-rubric version;
- task trials, successful/failed/aborted/censored counts, exclusion rules, latency samples or declared quantiles, and the uncertainty/statistical method;
- measured power-scope label kept explicit: display-only rail, rendering/GPU telemetry, gross whole-system mains power, or idle-subtracted system power;
- power-meter type/ID, calibration record and validity interval, accuracy/range, sampling frequency, monotonic-clock basis, timestamp units, integration method, maximum accepted sample gap and missing/duplicate sample counts;
- original power-trace digest, trace availability/access status, run start/end markers and task-window boundaries; preserve raw traces where rights permit;
- derived energy integral, denominator and joules/successful-task value with implementation/version, and an independent recomputation reference. A zero-success denominator is undefined, never smoothed or imputed;
- nominal luminance target and declared adaptation policy, actual emitted luminance as measured outcome, relevant contrast/readability/temporal floors, temperature and ambient conditions;
- applicable measurement-standard identifier/edition and explicit deviations.

Do not silently interpolate across a power-data gap beyond the predeclared bound. Missing trace data, unknown device/profile identity, invalid calibration, or an untraceable task denominator blocks a qualified energy claim. Negative energy saving is a valid regression and must remain visible. Task ratios are task-specific summaries, not an excuse to collapse task outcomes, latency, optical floors, energy and uncertainty into one opaque score.

For mains-powered computer monitors, record whether the method follows [IEC 62087-7:2018](https://webstore.iec.ch/en/publication/27420). For audiovisual test signals, record whether the stimulus method follows [IEC 62087-2:2023](https://webstore.iec.ch/en/publication/65117). Standardized stimuli and task-conditioned experiments are complementary; neither a custom task result nor a code-level reference implies formal conformity or certification.

### HDR transfer semantics

The current reference work includes a mathematical PQ decoder/encoder, while a future HLG profile is tracked in [Symtropy VIS-HDR-001 #1819](https://github.com/Luminous-Dynamics/symtropy/issues/1819). Do not label these as interchangeable “HDR gamma” curves: PQ's EOTF is an absolute display-referred code-to-luminance mapping, while HLG's OETF is scene-referred and its display-side rendering depends on explicit system/display parameters. Every derived value must state which transform and parameter set produced it.

An ST 2084 / PQ-decoded luminance value is a value computed from an encoded signal under a declared transfer function, not a physical measurement of a display. Store the signal input, PQ standard/edition, transfer implementation/version and computed cd/m² output as derived-analysis context. A physical display EOTF test must separately reference calibrated photometer/colorimeter readings, device mode, pattern/window size, sustained/peak duration, ambient conditions and instrument uncertainty. Do not let a correct PQ equation promote a hardware claim to measured or independently reproduced status.

### Manufacturing / microdisplay-yield evidence

For micro-LED AR microdisplay research, compose with [Symthaea MICRODISP-YIELD-001 #7868](https://github.com/Luminous-Dynamics/symthaea/issues/7868) and the [20-case synthetic workload manifest](https://github.com/Luminous-Dynamics/symthaea/blob/research/vis-tech-001-program-2026-10-17/docs/engineering/microdisplay-yield-corpus-v0.1.json). Catalogue these domain fields as evidence context; do not create a parallel manufacturing/yield authority:

- exact architecture, device/revision, array/process route, lot/run and stage IDs where available and disclosure permits;
- manufactured denominator, inspected denominator, inspection coverage, sampling plan, defect taxonomy and uninspected regions;
- stage-specific counts for emitter/transfer, placement, bond/interconnect, CMOS interface, optical assembly, test, repair and retest, without double-counting a repaired part;
- clear distinction among observed fraction, estimated probability, expected defect count and final good-unit yield;
- independence/correlation assumption and method for any composed yield or confidence interval;
- repair attempts/outcomes, post-repair measurements, calibration changes, scrapped/unknown units and residual defect maps;
- linked optical utility and reliability evidence (MTF, luminance/color, power/thermal state, viewing envelope, calibration and aging state), rather than a yield-only scalar;
- model/source/configuration digests and whether each quantity is synthetic, simulated, supplier-reported, bench-measured or independently reproduced.

Missing denominators or unknown inspection coverage block whole-population claims. Correlated/clustered defects must not be run through an independent Bernoulli interval without justification. A simulation may compare candidate yield-aware designs; only traceable production counts and qualified metrology support a physical yield claim. The literature's 99.99999% frontier is contextual research, not achieved capability or an automatic pass threshold.

### Scope and interpretation

- Exact claim text or stable claim ID, metric definition, applicable device/model/conditions and intended generalization range.
- Evidence class: analytic reference, synthetic, simulated, bench-measured, independently replicated or human-study.
- Raw metric vector and per-gate status; preserve uncertainty and tradeoffs instead of one opaque composite score.
- Limitations, known failures, discrepancy from the model and the next evidence required.
- Status is derived under the canonical evidence semantics; a producer-supplied PASS field cannot promote its own artifact.

### Decision-aware experiment evidence

For visual-domain design-of-experiments, compose with [Symthaea VIS-EXP-001 #7871](https://github.com/Luminous-Dynamics/symthaea/issues/7871) and shared [SE-OPT-BENCH-001 #7803](https://github.com/Luminous-Dynamics/symthaea/issues/7803). If a selector result is published, record the initial evidence snapshot, candidate design/configuration IDs, objective and hard-constraint version, uncertainty representation (probability model versus intervals/sets), available experiment actions, declared cost/time budget, selected or abstained action, and the exact hidden-oracle/evaluator version. Count failed, censored, missing and aborted experiments and their costs. Selection-policy outputs are derived recommendations, not physical evidence; a proposed measurement becomes physical evidence only after the actual instrument/run artifacts satisfy the applicable measurement contract.

### Rights and privacy

- License and redistribution state for raw and derived artifacts.
- Source URL/identifier, retrieval date, terms/version and local snapshot digest when permitted.
- Explicit data sensitivity, consent and retention/access limitations for scene imagery, eye tracking, face/body tracking, gaze traces or human-study records.
- Share derived aggregates rather than raw identifiable recordings by default.
- If rights or permissions are unknown, the artifact can be catalogued as restricted/unknown but must not be republished.

## Initial qualification fixtures

Build a compact, synthetic-only profile mapping test set:
- valid analytic fixture with expected units and input/output digests;
- synthetic output correctly labelled synthetic;
- simulated result cannot satisfy a bench-measured claim;
- bench result missing calibration is incomplete;
- digest mismatch and missing content object fail integrity;
- wrong device/model ID or swapped artifact-role digest is rejected;
- stale calibration, mismatched units or frame convention blocks the applicable claim;
- producer-declared PASS with missing raw output remains unqualified;
- two reports from the same pipeline are not automatically independent replication;
- source rights/redistribution unknown blocks public distribution;
- sensitive gaze/camera data defaults to restricted access;
- claim for one qualified device cannot generalize to all displays without additional evidence.

These are proposed tests; none is represented as already implemented or passing.

## Acceptance gates

- 100% correct mapping to existing artifact-kind/claim semantics for the frozen synthetic profile fixtures.
- Zero path from a producer-declared result directly to independently verified or physically measured status.
- Any digest mismatch, missing mandatory reference, stale calibration or wrong evidence class blocks the corresponding claim.
- Independent reproduction is linked to a distinct run and environment; disagreements remain visible.
- Synthetic, simulated, bench-measured and human-study classes remain distinct through query, export and publication.
- Scope expansion requires explicit new evidence; there is no universal "display passed" badge.
- Publication enforces rights/privacy policy and does not publish raw tracking/camera recordings by default.
- All results preserve metric vectors, denominators, known unknowns and caveats.

## Storage and protocol boundary

Do not store every frame or raw lab recording directly in a distributed ledger. Put large files in appropriate content-addressed or controlled storage and anchor only necessary typed references/digests and authorization/provenance metadata through existing Mycelix mechanisms. Availability of a digest does not guarantee byte availability; the profile should distinguish missing bytes from digest failure.

Do not create a new blockchain or consensus role for visual benchmarking. The value is shared evidence semantics, auditability, replication and bounded publication—not replacing the experiment or the instrument.

## Research references

- [VESA DisplayHDR performance criteria](https://displayhdr.org/performance-criteria/)
- [Khronos OpenXR Registry](https://registry.khronos.org/OpenXR/)
- [Meta-Optics for Optical Engineering of Next-Generation AR/VR Near-Eye Displays (2025)](https://doi.org/10.3390/mi16091026)
- [Micro-LED assembly/integration review (2025 online; 2026 issue)](https://doi.org/10.1088/2631-7990/ae25a8)
