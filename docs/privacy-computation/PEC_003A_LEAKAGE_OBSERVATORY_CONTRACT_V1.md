# PEC-003A — Privacy Leakage Observatory Contract v1

Status: measurement/evidence contract only

Tracks: #2121, #2109, #2110

Parent semantic waist: PEC-001A / #2116

## Purpose

Freeze an adversarial privacy-observation contract so Mycelix measures what each participant, verifier, server, colluding party, and network observer can learn instead of collapsing privacy to `private: bool`.

The governing rule is:

```text
no plaintext observed
    != no information leaked
    != unlinkability
    != anonymity
    != access-pattern privacy
    != privacy guarantee
```

PEC-003A adds no cryptographic implementation and establishes no privacy theorem.

## Closed v1 observation dimensions

The v1 observatory must be able to record observations for exactly these top-level dimensions:

```text
Identity
ParticipantCount
InputSize
OutputSize
SetSize
IntersectionSize
QueryIndex
AccessPattern
Timing
MessageSize
AbortBehavior
DropoutBehavior
CrossSessionLinkability
IdentifierReuse
TranscriptCorrelation
ResultValue
AuxiliaryInformationSensitivity
```

A later version may extend this vocabulary, but unknown dimensions must not be silently ignored by a profile claiming exhaustive coverage.

## Observer/adversary views

Every experiment binds one or more explicit observer views. The v1 role vocabulary is:

```text
LocalClient
RemotePeer
Coordinator
Verifier
Issuer
DatabaseServer
NetworkObserver
ColludingPartySet
CompromisedEndpoint
```

`ColludingPartySet` additionally binds the exact member/role set or an exact profile reference.

```text
hidden from server
    != hidden from network observer
    != hidden from colluding server + issuer
```

## Observation dispositions

For each dimension/observer pair, the observatory records one of:

```text
NotMeasured
NoDifferenceDetectedUnderExperiment
StatisticalSignalObserved
DeterministicallyRevealed
DerivedFromProtocolDefinition
Inconclusive
```

The dispositions deliberately avoid `Private` / `Secure`.

```text
NoDifferenceDetectedUnderExperiment
    != leakage absent
```

## Experiment identity

Every leakage experiment must bind exact:

- primitive/backend/profile identity;
- qualification state/reference if available;
- subject artifact/commit/program identity;
- workload/corpus identity;
- participant topology;
- adversary/observer model;
- network/execution environment;
- trial count and random-seed policy;
- measurement method/version;
- clock/timer source where timing is measured;
- serialization/transport profile;
- date/runtime/toolchain identity sufficient for reproducibility.

A measurement without an exact experiment identity is not admissible as PEC leakage evidence.

## Baselines and controls

Where a claim depends on statistical indistinguishability or reduced signal, experiments should bind an explicit baseline/control rather than interpreting a single distribution in isolation.

Examples:

```text
query A timings vs query B timings
set size n vs set size m message traces
same user across sessions vs different users
real protocol vs padded protocol
```

Control selection is theorem-bearing and must be recorded.

## Timing and message-size leakage

Timing and byte-count measurements must distinguish at least:

- application payload size;
- framing/transport overhead where observable;
- total messages/rounds;
- per-direction bytes;
- wall-clock latency;
- CPU/service time where independently measurable;
- jitter/noise assumptions.

```text
payload encrypted != payload length hidden
```

## Cross-session correlation

The observatory must support repeated-session experiments that vary context while holding relevant identities/workloads constant.

Potential correlation sources include:

- stable public keys;
- stable nullifiers or identifiers;
- ciphertext/proof size fingerprints;
- deterministic ordering;
- timing fingerprints;
- message counts;
- network endpoints;
- database access sequences.

```text
single-session privacy != cross-session unlinkability
```

## Abort/dropout leakage

Protocols may reveal information through whether, when, or how they abort.

Experiments should distinguish:

```text
honest completion
malformed-input abort
selective abort
party dropout
insufficient-threshold abort
backend resource-limit abort
```

An abort channel may itself be an oracle.

## Auxiliary-information sensitivity

Where applicable, experiments should record whether publicly available or attacker-supplied auxiliary information materially changes inference success.

Examples include known membership candidates, likely phone-number/email domains, public social graphs, known database size, known model architecture, or repeated observations.

PEC-003A does not define a universal inference model; exact attacker knowledge belongs in the experiment profile.

## Leakage receipt

A future `LeakageReceipt` should bind at minimum:

```text
experiment_profile_digest
subject_profile_digest
backend/profile identity
observer/adversary views
workload/corpus digest
execution capsule digest
measurement-method digest
per-dimension observations
raw-evidence reference/digest where retained
summary statistics
nonclaims
```

But:

```text
LeakageReceipt exists
    != privacy established
    != leakage exhaustively measured
    != theorem proven
    != production admission
```

## Negative-result language

The observatory must not translate an unsuccessful attack/measurement into an absolute privacy claim.

Preferred language:

```text
NoDifferenceDetectedUnderExperiment(E)
```

not:

```text
NoTimingLeakage
```

and:

```text
NoCrossSessionClassifierAboveThresholdUnderExperiment(E)
```

not:

```text
Unlinkable
```

unless a separate theorem/qualification establishes the stronger statement.

## Planner relationship

PEC-002 / Symthaea may consume qualified/admitted leakage evidence when comparing candidate privacy plans, but:

```text
better measured leakage profile
    != cryptographic superiority
    != universal privacy
    != production admission
```

Missing evidence is not equivalent to good privacy.

```text
NotMeasured != NoLeakage
```

A planner must be able to reject a candidate because a required leakage property is unknown.

## Domain relationships

The observatory is cross-cutting. Initial profiles should eventually cover:

- ZKP proof-size/timing/linkability observations;
- PSI set-size/result-size/enumeration/correlation observations;
- PIR repeated-query/timing/response-size observations;
- FHE ciphertext/key-size/evaluation-timing observations;
- MPC message-round/dropout/abort/collusion observations;
- federated-learning secure-compute composition observations.

No domain inherits a privacy PASS merely because it participates in the observatory.

## Required non-equivalences

```text
measurement != theorem
absence of detected signal != absence of leakage
encrypted payload != hidden metadata
single-session privacy != sequence privacy
single-observer privacy != collusion privacy
raw benchmark != qualified leakage evidence
qualified leakage evidence != application authorization
```

## Nonclaims

PEC-003A establishes no cryptographic security, anonymity, unlinkability, metadata privacy, access-pattern privacy, differential-privacy bound, side-channel freedom, exhaustive adversary coverage, legal/privacy compliance, production admission, application authority, or deployment readiness.
