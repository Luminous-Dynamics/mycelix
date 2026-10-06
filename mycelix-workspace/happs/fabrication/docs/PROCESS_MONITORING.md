# Fabrication Process Monitoring (FPM)

Fabrication Process Monitoring (FPM) is the canonical name for the Fabrication hApp's in-process manufacturing monitoring subsystem.

The historical Cincinnati name remains a compatibility/codename reference only. It is not the name of a proven detector algorithm.

## Evidence layers

FPM deliberately separates four layers:

1. Observation — measured process data such as temperature, motor current, vibration, and filament tension.
2. Detection — a deterministic inference that an observation differs from a declared baseline.
3. Intervention — a proposed or executed response to a detection. The first FPM engine does not perform interventions.
4. Assessment — a quality or defect conclusion about the produced part. This requires a separate, versioned assessment model and independent validation.

A detector result is therefore not, by itself, proof that a physical defect exists.

## Deterministic FPM core

The initial Rust implementation lives in the fabrication_common crate as the fpm module.

Its core pipeline is:

ProcessObservation -> BaselineProfile -> analyze() -> ProcessDetection

### Baseline

FPM uses robust median and median-absolute-deviation (MAD) statistics for the initial detector.

Required channels must have at least 5 baseline observations:

- hotend temperature;
- bed temperature;
- extruder current;
- vibration RMS.

Filament tension is optional and is only admitted into a baseline after the same minimum sample count is available.

The baseline records its schema version, detector identifier/version, source observation count, source-observation SHA-256 commitment, and feature statistics.

### Detection semantics

| Feature | Inference class |
|---|---|
| Hotend temperature | TemperatureDeviation |
| Bed temperature | TemperatureDeviation |
| Extruder current | ExtrusionInconsistency |
| Vibration RMS | VibrationAnomaly |
| Filament tension | FilamentSlip |

The detector emits an anomaly_score in [0, 1]. This value is a normalized decision score, not a probability, confidence, or physical severity measurement.

Zero-dispersion baselines are handled explicitly: an exact match is normal; any differing observation is anomalous, with no fabricated z-score.

## Provenance

Every ProcessDetection records:

- FPM schema version;
- detector identifier;
- detector version;
- SHA-256 digest of the baseline;
- SHA-256 digest of the observation;
- SHA-256 digest of the complete analysis input.

This means the same detector can be re-run against the exact committed inputs rather than relying on mutable runtime state.

The pure FPM engine does not read the DHT, call other zomes, read wall-clock time, or modify printer state.

## What is not claimed

FPM v1 does not claim:

- calibrated defect probabilities;
- general defect-detection accuracy;
- universal sensor thresholds;
- physical defect confirmation from a sensor anomaly alone;
- safe autonomous printer control.

Those require separate empirical qualification.

## Qualification direction

The first qualification corpus should contain at least:

- healthy traces;
- isolated sensor spikes;
- gradual sensor drift;
- correlated multi-sensor deviations;
- missing observations;
- duplicated observations;
- reordered observations;
- timestamp/sequence discontinuities;
- calibration/profile changes;
- detector-version changes.

Physical claims should additionally use independent reference measurements such as dimensional inspection, microscopy, CT/XCT, or other process-appropriate nondestructive/destructive tests.

## Standards context

ISO/ASTM 52953:2025 establishes general requirements for registration of process-monitoring and quality-control data in additive manufacturing and explicitly addresses multimodal data registration.

ISO/ASTM TR 52958:2026 provides a concrete in-situ monitoring and flaw-detection workflow using coaxial photodiode monitoring, statistical methods, clustering, seeded flaws, and CT-based validation.

ASTM E3353 covers in-process monitoring sensors, configurations, data analysis, and data uses for laser powder bed fusion.

The FPM architecture therefore treats data registration, inference, and physical validation as distinct concerns instead of treating a sensor anomaly as a finished quality verdict.

## Migration

Legacy Cincinnati types and APIs should not be renamed blindly. Once the FPM schemas have been qualified, the naming migration can introduce new canonical types while retaining explicit deserialization compatibility for existing records.
## Exact contextual baseline boundary

FPM context binding is implemented separately from the raw detector so that a statistically unusual value is not confused with a context change.

A `ProcessContext` records, when available, the process phase, layer, move class, commanded speed, target temperatures, acquisition clock domain, sensor source identity, and calibration-profile commitment. Baseline construction requires the source observations to carry one exact common context.

The contextual selector is deliberately **exact-match only** in v1:

- exact context -> detector may run;
- context mismatch -> explicit `ContextMismatch` error;
- missing required registration/discriminator -> explicit `IncompleteContext` error;
- mixed-context baseline source window -> rejected rather than pooled.

There is no implicit nearest-context, global-baseline, or "best effort" fallback. Explicit compatibility rules can be added later as a separately versioned policy, but they must never be inferred from statistical similarity.

The contextual detection provenance records both the context commitment and a combined digest over the detector result and context. This keeps raw detector evidence distinguishable from the stronger claim that the baseline was selected under an exact registered process context.

## Registration and physical validation boundary

Context binding is necessary but not sufficient for multimodal registration or physical quality claims. ISO/ASTM 52953:2025 establishes requirements for registering process-monitoring and quality-control data, including multimodal data registration. ISO/ASTM TR 52958:2026 describes a flaw-detection workflow that uses process monitoring together with seeded flaws and CT validation. These support keeping data registration, inference, and independent physical validation as separate evidence layers. citeturn831155search0turn831155search1

The next research layer is therefore a separately versioned multimodal registration contract covering source-local clocks, alignment, calibration/profile identity, and explicit handling of missing or conflicting context. It should not silently convert alignment uncertainty into a normal or healthy result.
## Multimodal registration evidence

FPM now has a separate registration layer for heterogeneous sensor evidence. A `RegistrationEnvelope` binds source identity, modality, source-local sequence/timestamp, clock domain, an explicit correlation domain and producer-assigned correlation ID, calibration-profile commitment, process-context commitment, source-data digest, and an explicit alignment method.

The registration envelope also requires at least two distinct modalities and rejects duplicate `(source_id, modality)` participant identities. This prevents a single modality from manufacturing a superficially multimodal registration by repeating the same source record.

Registration state is derived from the envelope rather than accepted as a caller-supplied field. Correlation IDs are namespace-bound so an opaque ID reused by unrelated acquisition systems cannot by itself imply shared frame identity. The mechanically derived positive state is named `Consistent`, not `Registered`, because these inputs remain supplied by the acquisition/registration layer:

- **Consistent** — supplied registration metadata is internally coherent and passes the deterministic checks;
- **Unregistered** — required registration/alignment evidence is absent;
- **Conflicting** — participants disagree on context, calibration, clock/sequence, or timestamp alignment;
- **Invalid** — the envelope/schema/digest is malformed;
- **Unknown** — evidence is insufficient to establish the requested alignment.

The first exact methods are correlation-ID alignment and shared-clock exact source timestamp. A correlation ID is a producer-assigned identifier for the same acquisition frame within an explicit correlation domain; source-local sequence counters are not assumed comparable across modalities. Cross-clock registration requires an explicit committed transform or external registration-evidence digest. The presence of such a commitment records an explicit registration claim; it does not independently prove the transform, sensor calibration, or physical truth.

This distinction follows established industrial data semantics: OPC UA preserves a source timestamp assigned at the data source and distinguishes it from server receipt time; it also calls for synchronized source clocks where sources are redundant. IEEE 1588/PTP is a standard mechanism for synchronizing clocks in networked measurement and control systems. citeturn106905search0turn106905search3turn106905search6

A downstream detector should consume multimodal evidence only after the registration state and provenance requirements for that detector are satisfied. Registration status must never be silently converted into a Normal detection result.

## Registration qualification

Registration consistency is not the terminal trust boundary. FPM now has a deterministic qualification layer that consumes an exact `RegistrationEnvelope` together with the immutable bytes referenced by its commitments.

The structural profile records:

- the exact registration-envelope digest;
- exact source-data, calibration-profile, process-context, and applicable alignment-evidence commitments;
- a deterministic evidence-manifest digest;
- an explicit qualification profile identifier/version/digest;
- an explicit verifier identifier/version.

The verifier re-hashes resolved evidence itself. Missing, substituted, duplicated, unexpected, or cross-kind artifacts cannot produce `QualifiedForProfile`.

`QualifiedForProfile` is deliberately narrower than verification. It does not prove source authenticity, clock synchronization, calibration correctness, provenance independence, or physical truth. In particular, distinct modality labels establish modality diversity only. A stronger acquisition-provenance profile must bind participant metadata to committed source-observation records and independently qualify acquisition ancestry; this is tracked separately in #4331 and #4333.

The qualification core is deterministic: it does not read wall-clock time, query the DHT, depend on mutable host state, or trust a serialized verification boolean. The resulting record is therefore suitable as a stable evidence object for a later authenticated/transparency layer.

## Anchored qualification and replay resistance

The original structural qualification profile (`fpm.registration.structural`) proves exact internal consistency and exact commitment closure, but it is intentionally **not** an authority or anti-replay proof. A caller who supplies a different self-consistent envelope and recomputes its digest can produce a new structural qualification.

The anchored profile (`fpm.registration.anchored`) therefore requires an external registration-anchor binding whose committed envelope digest exactly matches the evaluated envelope. This makes the dependency explicit, but it is **not itself an authority or anti-replay proof**: the pure qualification core still does not authenticate the anchor reference. That responsibility belongs to the Holochain/external resolver tracked in #4354.

For Holochain integrations, preserve the distinction between:
- `EntryHash`: content identity for the registration entry;
- `ActionHash`: identity of the authored entry instance, carrying source-chain/timestamp context.

The authority-resolution layer now has concrete verification entry points in the Verification coordinator:
- `create_fpm_registration_anchor` stores a typed public registration-anchor entry containing the exact envelope plus its recomputed digest;
- `resolve_fpm_registration_action_anchor` retrieves `get_details(ActionHash, network)`, requires `ValidationStatus::Valid`, rejects updates/deletes, requires the addressed action itself to be `Create`, decodes the typed anchor, and recomputes the envelope digest before returning author/timestamp evidence;
- `resolve_fpm_registration_entry_anchor` retrieves `get_details(EntryHash, network)`, requires a live entry with no rejected creation actions, updates, or deletes, confirms the returned record still carries the requested `EntryHash`, and recomputes the envelope digest.

The EntryHash resolver intentionally returns no author/timestamp claim: an `EntryHash` is content identity, whereas an `ActionHash` identifies an authored instance and carries source-chain context. The ActionHash result preserves author, signer, timestamp, action sequence, and previous-action hash separately, and callers may bind an expected author and/or signer explicitly. Holochain documents these action fields and the distinction between author and signer. citeturn410340search1turn388535search0

This adapter is still not a claim of physical truth or universal authority: Holochain `get` only establishes that the local/returned view considers the resolved record valid, and other validators can have different information. The resolver consequently treats validation state, deletion/update metadata, typed entry decoding, and exact envelope commitment as separate predicates rather than collapsing them into a single “trusted” bit. citeturn832284search0

Verifier identity is treated similarly. `verifier_id`, `verifier_version`, and `declared_verifier_implementation_digest` are captured declarations in the pure core, not proof of the binary that executed. Authenticated build/execution identity is a separate boundary tracked in #4355.

## Authenticated acquisition roots

The authenticated provenance layer now requires each distinct `acquisition_root_digest` to resolve to its own typed Holochain acquisition-root anchor.

An acquisition-root anchor binds:
- a source-system identifier;
- an immutable capture reference;
- an artifact digest;
- a deterministic root commitment derived from those fields.

Resolution requires a valid original Create action, no updates or deletes, the exact acquisition-root entry type, canonical commitments, and recomputation of the root commitment. Authenticated provenance qualification rejects missing root coverage, extra root anchors, and duplicate root commitments.

This is intentionally a bounded trust step rather than a physical-truth claim. The root anchor proves that a particular Holochain action committed a particular capture-reference/artifact-digest declaration. It does not independently verify the referenced artifact, the honesty of the source system, or the physical independence of the acquisition device. Those require a separate source-system or metrological trust root.

## Authenticated provenance

The provenance-disjointness qualifier evaluates the supplied lineage graph, but a producer could still fabricate that graph unless the individual provenance declarations have an independent integrity anchor.

FPM now adds a separate authenticated provenance layer:

- each provenance witness is committed as a typed `FpmProvenanceAnchor` Holochain entry;
- the entry binds the witness to a specific registration `ActionHash`;
- the witness digest is recomputed from the stored witness rather than trusted as a label;
- resolution requires the provenance action to be a valid original `Create` action with no updates or deletes;
- resolution requires the exact FPM provenance entry type;
- the referenced registration `ActionHash` is independently resolved and its envelope digest is recomputed;
- the witness's source/modality pair and source-observation binding must match that exact registration envelope;
- only after every supplied provenance anchor has been resolved does FPM feed the witnesses into the existing provenance-disjointness qualifier.

The resulting qualification preserves each provenance anchor's ActionHash, author, signer, timestamp, source-chain sequence, previous-action hash, and witness commitment as separate evidence. The complete authenticated qualification also receives its own deterministic manifest and record digest.

This establishes **authenticated provenance declarations**: who committed which lineage statement, which registration anchor it referenced, and whether the declaration remains valid in the resolver's Holochain view. It does not establish that the declared sensor lineage is physically true, that the source device is honest, or that two sources are physically independent.

This distinction mirrors supply-chain provenance systems such as in-toto/SLSA, where a provenance statement identifies a subject and describes production lineage but still depends on the trustworthiness of the attesting builder/platform. citeturn173650search0turn173650search6

## Provenance disjointness qualification

FPM can now qualify a supplied acquisition-lineage graph separately from registration consistency. A provenance witness binds a participant to its exact source-observation commitment, an acquisition-root commitment, and explicit parent lineage nodes.

The provenance-disjointness profile fails closed for:
- missing participant witnesses;
- duplicate participant/node/parent relationships;
- malformed or mismatched commitments;
- missing parents;
- lineage cycles;
- shared acquisition roots;
- shared lineage ancestry;
- excessive graph cardinality.

This result is still a statement about the **supplied provenance graph**. It does not prove that the real sensors were physically independent, honest, or correctly identified. Those properties require authenticated provenance anchored to the underlying acquisition system.

The intended evidence chain is therefore:

`Observation → RegistrationConsistency → SourceObservationBinding → ProvenanceDisjointness → Authenticated Provenance → Clock/Calibration Qualification → PhysicalValidation`

W3C PROV similarly models provenance in terms of entities, activities, and relationships among them; the current FPM graph is an intentionally smaller deterministic subset focused on acquisition ancestry. citeturn874737search6

## Next research boundary

The remaining work is empirical qualification of registration and detection against captured process traces, including clock skew, dropped/duplicated samples, calibration changes, context conflicts, and independent physical reference measurements. The current implementation remains a deterministic evidence contract, not a validated physical-defect detector.

