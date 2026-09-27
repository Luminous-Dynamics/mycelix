# MYC-INT-018J — H1 Wet-Bench Productive-Node Reference Design

Status: physical reference architecture only. Tracks #3239. Child of MYC-INT-018I / PR #3238.

Observed design date: 2026-09-27.

## 1. Purpose

H1 is the first physical productive-node stage after the synthetic H0/018H corpus.

It exists to prove that real instruments, plumbing, power state, failures, restarts and physical observations can enter the Mycelix/Integral evidence architecture without collapsing:

```text
raw measurement
!= derived assessment
!= analysis
!= decision
!= authorization
!= effect
```

H1 is **not** yet an agronomic performance trial. It does not need plants in order to be useful.

## 2. H1 progression

Use three independently reviewable stages.

### H1a — dry instrumentation / power bench

No circulating water.

Goals:

- controller boot/restart behavior;
- sensor discovery/identity;
- power/energy measurement;
- manual override and stop behavior;
- logging/export;
- synthetic sensor-frame injection;
- timestamp/currentness handling;
- failure-state display.

### H1b — clean-water closed loop

Use a benign water loop to test physical hydraulics and wet-side instrumentation.

Goals:

- pump state versus measured flow;
- reservoir level;
- flow restriction;
- leak/spill evidence;
- temperature observations;
- restart/reconnect under a real physical process;
- duplicated/out-of-order delivery;
- energy-versus-flow observations.

### H1c — nutrient-solution sensing profile

Introduce the solution chemistry needed to exercise hydroponic measurement semantics.

Goals:

- pH instrumentation;
- electrical conductivity instrumentation;
- optional dissolved oxygen for profiles where it is physically relevant;
- solution temperature;
- chemistry-currentness and disagreement cases;
- derived-assessment boundary.

H1c still has **no automated dosing authority**.

## 3. Reference hydraulic topology

```text
                  +----------------------+
                  |      reservoir       |
                  | level / temp /       |
                  | pH / EC / optional DO|
                  +-----------+----------+
                              |
                              v
                     +----------------+
                     | low-voltage DC |
                     | circulation    |
                     | pump           |
                     +--------+-------+
                              |
                              v
                       flow observation
                              |
                              v
                  +----------------------+
                  | instrument / test    |
                  | manifold             |
                  |                      |
                  | normal path          |
                  | restriction fixture  |
                  | bypass/service path  |
                  +-----------+----------+
                              |
                              v
                           return
                              |
                              +----------> reservoir

spill tray / leak sensor observes the wet assembly independently
```

The test manifold is intentionally serviceable so H1 can inject partial restriction without damaging the pump or requiring a crop channel.

## 4. Essential physical distinctions

The bench must make these disagreements observable rather than treating them as impossible states:

```text
pump commanded ON
!= pump electrically energized
!= water flowing
!= adequate water flow

reservoir level present
!= source water admitted
!= solution chemistry suitable

pH value present
!= pH sensor calibrated
!= pH current
!= process suitable

energy consumed
!= useful hydraulic work
```

## 5. Measurement classes

### 5.1 Root-zone / solution channels

Required H1 classes:

- solution temperature;
- pH;
- EC / conductivity;
- reservoir level or volume observation;
- flow rate;
- pump commanded state;
- pump electrical/energized state where available.

Conditional classes:

- dissolved oxygen for DWC/aerated or other profiles where DO is a meaningful process coordinate;
- make-up-water volume;
- pressure where the selected plumbing/control profile needs it.

Do not infer an absent channel.

```text
no DO sensor
!= adequate DO

no pressure sensor
!= nominal pressure
```

### 5.2 Aerial / greenhouse-compatible channels

H1 should define the profile even if not all channels are installed until H2:

- air temperature;
- relative humidity;
- light under an explicit measurement profile (for example PPFD rather than a vague `light_level`);
- optional CO2;
- later canopy temperature when a crop exists.

These are separate from solution/root-zone measurements.

### 5.3 Energy / reliability channels

- pump/subsystem electrical consumption or power;
- controller power state;
- boot/restart event;
- communication/link state where observable;
- leak/spill detector state;
- local manual override state;
- physical stop state where instrumented.

## 6. Research basis and scope

Cornell Controlled Environment Agriculture materials identify greenhouse air temperature, relative humidity, CO2, light, nutrient-solution temperature, pH, dissolved oxygen and electrical conductivity as monitored controlled-environment parameters.

Cornell fault-detection work also provides a useful architecture precedent in which nutrient-solution variables are monitored as a root-zone subsystem while the aerial environment is handled separately, with monitoring occurring more frequently than persistent logging.

H1 consumes that only as support for **measurement classes and subsystem separation**.

It does not copy crop-specific setpoints or claim that one sampling cadence is universally correct.

## 7. Acquisition profiles

Sampling frequency is versioned configuration, not semantic truth.

Recommended first engineering profile:

### Class A — fast hydraulic/electrical state

Examples:

- flow;
- pump state;
- leak detector;
- instantaneous power if available.

Candidate local sampling: approximately 1 Hz where supported.

### Class B — chemistry/environment

Examples:

- pH;
- EC;
- solution temperature;
- DO;
- air temperature/RH.

Candidate local sampling: every 5–15 seconds.

### Class C — canonical observation/log projection

Candidate interval: 1–5 minutes for ordinary persistent summaries, while transitions/faults remain event records and raw local samples may be retained according to profile.

These numbers are H1 defaults only.

```text
acquisition cadence v1
!= acquisition cadence v2
```

A cadence change that affects interpretation must produce a new acquisition/measurement profile identity.

## 8. Instrument identity and calibration evidence

For every installed channel preserve as available:

```text
instrument identity
model/profile identity
measurement subject/location
unit/profile
installation generation
calibration/check method ref
calibration/check time evidence
calibration result/acceptance evidence
precision/uncertainty metadata
source/currentness policy
replacement/supersession lineage
```

Do not infer calibration from the product/model name.

```text
instrument supports calibration
!= calibration performed

calibration performed once
!= calibration current forever
```

H1 does not define one universal calibration interval. Each instrument/profile owns that evidence horizon.

## 9. Observation admission

Once 018F's upstream identity dependencies qualify, a physical reading should enter the interoperability layer conceptually as:

```text
native sensor/domain record
        +
subject + measurement profile
        +
source/provenance/time evidence
        ↓
PhysicalObservationBinding
```

Until then, H1 run data must be labelled as experimental/source data rather than pretending to instantiate qualified EPI/018F identities.

## 10. Assessment boundary

H1 deliberately creates situations where a value exists but no positive assessment follows.

Examples:

```text
pH observation
!= hydroponic suitability

water-quality convenience flag
!= independent standards assessment

flow observation
!= adequate-flow assessment

energy meter value
!= verified-energy assessment
```

018G-style assessment fixtures may consume exact observation refs after their identity/evidence owners qualify.

## 11. Symthaea boundary

When SYM-INT-001B qualifies, H1 may export a bounded read-only `AnalysisRequest`.

Potential H1 analytical tasks:

- detect pump-command/flow inconsistency;
- detect possible slow restriction from flow/energy change;
- identify stale/disagreeing sensor evidence;
- estimate whether an observed pattern deserves review;
- compare pre/post intervention evidence.

Always:

```text
AnalysisArtifact
!= observation
!= controller command
!= pump authority
!= dosing authority
```

The current `symthaea-agribot` state is soil/open-field shaped and must not receive hydroponic values by relabeling soil channels. Hydroponic channels need an explicit profile/adaptor.

## 12. Actuation posture

H1 is observation-first.

Allowed first-generation control paths:

### H1a

- manual local switching;
- bounded test commands under operator presence;
- physical stop/override.

### H1b/H1c

- pump on/off for controlled bench runs;
- optional bounded service/test valve operation only if separately instrumented and safe.

Not in H1:

- automated nutrient dosing;
- automatic pH adjustment;
- autonomous climate control;
- autonomous crop treatment;
- Symthaea-to-actuator path;
- remote unsupervised physical control.

## 13. Electrical / wet-side safety boundary

Prefer extra-low-voltage DC devices on the wet side of the bench.

Mains-powered equipment near water is facility electrical work, not a software feature. Where mains is unavoidable, require applicable jurisdictional protection and qualified installation, including appropriate enclosure/splash protection, earthing/grounding, overcurrent protection and residual-current/GFCI-equivalent protection.

Use:

- drip loops;
- strain relief;
- splash separation;
- spill containment;
- clear wet/dry zones;
- accessible physical isolation/stop;
- fused/protected low-voltage branches as appropriate.

```text
software interlock
!= electrical safety certification
```

## 14. Vendor-neutral BOM classes

### Hydraulic

- 1 opaque serviceable reservoir, bench-scale;
- 1 low-voltage circulation pump;
- compatible tubing/fittings;
- serviceable restriction/bypass section;
- return fitting/manifold;
- spill tray/secondary containment.

### Root-zone/process instrumentation

- 1 flow sensor;
- 1 level sensor or independent volume measurement method;
- 1 solution-temperature probe;
- 1 pH instrument/probe;
- 1 EC/conductivity instrument/probe;
- optional DO instrument for selected profile;
- 1 leak detector.

### Ambient

- 1 air-temperature/RH sensor;
- optional explicit-profile light sensor for H1/H2 preparation;
- optional CO2 instrument later.

### Energy/control

- 1 edge acquisition/controller device;
- appropriate isolated low-voltage supply;
- protected switching appropriate to the pump/load;
- subsystem power/energy observation method;
- local manual control;
- physical stop/override;
- visible status indication.

### Evidence/service

- calibration/check materials appropriate to selected instruments;
- labels/asset IDs;
- cable management;
- service log;
- replacement/spare consumables appropriate to probes.

No brand/model is part of the protocol identity.

## 15. Reference build-size principle

H1 should be deliberately small enough that:

- the complete hydraulic loop is visible;
- a spill is containable;
- sensors can be removed/replaced without crop loss;
- reservoir contents can be changed safely;
- the system can be power-cycled repeatedly;
- every channel can be independently inspected;
- one person can reproduce a run.

Do not optimize H1 for farm throughput.

## 16. Failure-injection matrix

### FI-01 missing sensor

Disconnect or omit a channel.

Expected:

```text
missing
!= zero
!= normal
```

### FI-02 stale stream

Stop updates while retaining the last value.

Expected: old value remains historical; currentness becomes stale/unknown under profile.

### FI-03 stuck-at value

Feed a constant stream despite physical change.

Expected: source data remains source data; a derived fault assessment may flag inconsistency but must not rewrite the observation.

### FI-04 controlled drift fixture

Simulate or replay gradual bias.

Expected: drift evidence and assessment remain distinct from raw input.

### FI-05 duplicate delivery

Deliver the same observation more than once.

Expected: multiple deliveries, one semantic observation where exact identity matches.

### FI-06 out-of-order arrival

Expected: arrival order does not become observation/event-time truth.

### FI-07 conflicting probes

Expected: preserve both; no silent average-as-truth.

### FI-08 pump command without flow

Expected:

```text
commanded ON
+ measured no/low flow
```

remains representable.

### FI-09 partial restriction

Expected: flow degradation evidence without inventing cause.

### FI-10 low reservoir

Expected: level observation/finding does not itself grant refill authority.

### FI-11 leak event

Expected: leak observation/finding; physical safety response may be handled by a local hardwired/safety profile separately from Symthaea.

### FI-12 restart/power cycle

Expected: restart does not duplicate historical physical events or fabricate missing interval data.

### FI-13 network partition

Expected: local acquisition may continue under profile; reconnect preserves original observation identities and delivery history.

### FI-14 timestamp skew

Expected: source timestamp is evidence, not trusted global time.

### FI-15 missing energy meter

Expected: energy becomes unknown, not zero.

### FI-16 manual override

Expected: manual operation is recorded separately from automated/requested state where observable.

### FI-17 unauthorized actuation attempt

Expected: no effect admission through analysis/assessment paths.

### FI-18 profile revision mid-run

Expected: old/new measurements retain exact profile generations rather than being silently reinterpreted.

## 17. Run manifest

Every physical H1 run should eventually preserve at minimum:

```text
run_id
H1 stage/profile generation
bench design/profile ref
installed instrument inventory
instrument generations
measurement/acquisition profiles
calibration/check evidence refs where available
operator/run initiator
start/end time evidence
controller/software version
source data commitment/export ref
failure-injection plan + actual injections
manual interventions
power/restart events
known limitations
synthetic/replayed input markers
physical observation summary refs
```

A dashboard is never the run record.

## 18. Physical versus synthetic provenance

H1 mixes real and synthetic/replayed inputs during fault testing.

The provenance label must therefore be load-bearing.

```text
physical sensor observation
!= synthetic frame
!= replayed historical frame
!= derived fault injection
```

No visualization/export step may drop this distinction.

## 19. Data export requirement

An H1 run must remain interpretable when:

- Symthaea is disabled;
- ITC is disabled;
- FRS summaries are discarded;
- the dashboard implementation changes;
- PostgreSQL/Holochain projection choices change.

Source evidence and profile identities survive those replacements.

## 20. H1 exit gate

H1 exits only when:

1. every installed channel has explicit subject, unit and measurement-profile identity;
2. missing/stale/conflicting states remain explicit;
3. restart/reconnect does not fabricate or silently rewrite observations;
4. pump commanded/energized/measured-flow states can disagree safely in the data model;
5. chemistry measurements cannot become suitability/safety assessments implicitly;
6. source-domain convenience flags cannot satisfy qualified assessment requirements;
7. synthetic/replayed injections cannot become physical observations;
8. physical run provenance/export is retained;
9. read-only analysis cannot authorize physical effects;
10. the run remains semantically interpretable without Symthaea and ITC;
11. no instrumentation PASS is presented as agronomic, food-safety or commercial PASS.

## 21. What H1 unlocks

After semantic dependencies and H1 instrumentation qualify, H2 can add:

- a small crop rack / greenhouse bay;
- production-batch lineage;
- harvest/output observations;
- crop-health/canopy channels;
- hydroponic versus soil/reference comparison;
- preregistered outcome expectations;
- read-only Symthaea hydroponic analysis profile.

H2 should reuse the exact H1 measurement profiles where semantics have not changed.

## 22. Nonclaims

H1 does not establish:

- universal hydroponic operating setpoints;
- instrument accuracy merely because a reading exists;
- current calibration merely because calibration is supported;
- potable-water or hydroponic-process suitability;
- crop health/yield improvement;
- food safety;
- commercial performance;
- electrical certification;
- autonomous agricultural safety;
- governance/economic superiority;
- qualified Symthaea control authority.
