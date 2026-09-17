# REGEN-044 — Repair and Skill Dependency Projection v1

Status: preregistration only.

This contract defines how regenerative resilience analysis may represent repair, maintenance, calibration, inspection, operator competence and recoverability dependencies without equating equipment ownership or nominal spare inventory with recoverable service.

It consumes the service denominator from REGEN-040, dependency closure from REGEN-041, compound-shock state from REGEN-042, and qualified substitution frontier from REGEN-043.

## 1. Core theorem

```text
failed/degraded capability
+ exact repairable subject
+ exact failure state
+ qualified diagnosis
+ compatible parts/tools/software
+ qualified skill/authority
+ time/access/environment
= repair/recovery candidate
```

not:

```text
spare part exists
= repair possible

technician exists
= technician available/qualified

equipment repaired
= service restored
```

## 2. Repair is capability-relative

A repair statement must bind the exact capability or service dependency being recovered.

The same component may matter differently for:

- soil processing;
- water service;
- storage/cold chain;
- food processing;
- biochar/compost operations;
- laboratory/testing;
- transport;
- communications;
- energy conversion;
- monitoring/sensing.

No global `repairable=true` property is sufficient.

## 3. Asset identity and configuration are explicit

A repair candidate must bind the exact asset/configuration revision it concerns where known.

```text
same product family
!= same compatible part
!= same firmware/configuration
!= same repair procedure
```

Model/serial/revision/installed-option differences must remain representable through exact references rather than inferred from names.

## 4. Failure state is first-class

The repair model must distinguish at least:

- operational;
- degraded;
- failed;
- isolated/unsafe-to-operate;
- diagnosis unresolved;
- repair in progress;
- repaired pending verification;
- recovered/returned-to-service.

```text
repair completed
!= verified return to service
```

## 5. Diagnosis is not repair

The model must preserve:

```text
symptom observation
!= fault hypothesis
!= confirmed diagnosis
!= repair plan
!= repair execution
!= verified recovery
```

Symthaea may assist diagnosis and plan comparison but cannot convert confidence into authorization or return-to-service evidence.

## 6. Skill is not a Boolean

A skill dependency should be scoped by:

- capability/task;
- equipment/system family;
- proficiency/qualification evidence where applicable;
- authorization/licensing where applicable;
- recency/currentness;
- availability window;
- location/travel dependency;
- supporting tools/information.

```text
person once trained
!= currently qualified
!= currently available
```

## 7. Single-person dependency is visible

If one service or repair path depends on one person, that is a first-class concentration risk.

The system should expose:

- person concentration;
- shift/time availability;
- travel/access dependency;
- communication dependency;
- credential/authority dependency;
- knowledge-transfer status;
- succession/training coverage.

A community should not receive a false “local capability” claim when the capability disappears if one individual is unavailable.

## 8. Documentation is a dependency

Repairability may depend on:

- service manual;
- wiring/schematic information;
- calibration procedure;
- firmware/configuration backup;
- diagnostic codes;
- parts catalog;
- safety procedure;
- inspection/acceptance criteria.

Documentation availability, version and access rights are explicit dependencies.

```text
technician + tools
!= repairability when required knowledge is unavailable
```

## 9. Tooling is explicit

The repair path may depend on ordinary or specialized tools, fixtures, test equipment, lifting/handling resources, calibration devices, clean/dry environments or software tools.

Tool presence alone does not imply calibration or fitness for use.

## 10. Parts inventory is exact and stateful

Spare parts must preserve:

- exact identity/compatibility;
- quantity;
- storage state where relevant;
- expiry/shelf-life where applicable;
- reservation/commitment;
- installation consumption;
- replenishment source;
- lead time.

One spare cannot be counted simultaneously for two incompatible repair plans.

## 11. Cannibalization is explicit

Recovering one asset by consuming components from another may be a legitimate candidate strategy, but it must record the capability sacrificed.

```text
asset A recovered via part from asset B
!= net capability gain by definition
```

The resulting service tradeoff must remain visible.

## 12. Software/firmware is a repair dependency

Where a repair requires software, configuration or firmware, the model should preserve:

- exact version/artifact identity;
- integrity/provenance;
- compatible hardware revision;
- signing/authorization requirement;
- offline availability;
- configuration backup;
- rollback/recovery path.

A cloud-hosted installer or vendor portal is a dependency and must appear in closure analysis.

## 13. Supply-chain depth is explicit

“Part available locally” is not sufficient if replenishment requires a fragile upstream chain.

The model should distinguish:

```text
part on hand
local fabrication/repair capability
regional supply
external supplier
single-source supplier
replenishment lead time
```

These are different resilience propositions.

## 14. Fabrication is not automatic substitutability

Local fabrication capability does not automatically imply that a replacement part is safe, compatible or fit for service.

The model must separate:

```text
can fabricate geometry
!= can fabricate qualified replacement
```

Where verification/testing is required, it remains a hard prerequisite.

## 15. Calibration and verification are distinct

Some recovered capabilities require calibration or acceptance testing before return to service.

Therefore:

```text
mechanically restored
!= calibrated
!= verified
!= authorized for return to service
```

The authoritative domain owns the applicable acceptance requirements; REGEN-044 references them.

## 16. Repair authority remains external

Technical ability does not imply permission to repair, modify, configure, inspect, certify or return an asset to service.

This contract creates no professional license, manufacturer authority, legal right, safety waiver or emergency authorization.

## 17. Time-to-recovery is decomposed

Recovery time should expose components such as:

- detection delay;
- diagnosis delay;
- access/travel delay;
- parts/tool acquisition;
- repair duration;
- calibration/verification duration;
- authority/approval delay;
- restart/reintegration time.

A single opaque MTTR value may be recorded as evidence, but the deterministic resilience model should not assume it explains every state transition.

## 18. Recovery windows are service-relative

A repair completed after the service's critical continuity window may still reduce long-term outage but does not retroactively preserve continuity.

```text
recovered eventually
!= continuity preserved
```

## 19. Preventive maintenance is separate from repair

The model must distinguish preventive/predictive maintenance from corrective repair.

Maintenance may reduce risk or extend availability, but this contract does not treat maintenance completion as proof that failure probability is zero.

## 20. Deferred maintenance remains visible

Unresolved maintenance obligations may affect readiness, but the model must not invent a universal penalty or failure probability absent evidence.

The state should remain inspectable rather than compressed into a hidden risk score.

## 21. Human fatigue and workload are dependencies

During compound shocks, the same people may be needed for operations, repair, transport, care work, governance, communications and emergency response.

The model should permit explicit competing commitments rather than assuming unlimited skilled labor.

## 22. Training capacity matters

Long-term local resilience may improve when a skill can be reproduced locally.

The model should distinguish:

- current qualified personnel;
- trainees;
- instructors/mentors;
- training material;
- training equipment;
- supervised practice requirement;
- time to independent capability.

```text
training program exists
!= trained capacity exists now
```

## 23. Knowledge replication is not authority replication

Documentation/training may reproduce knowledge while licenses, organizational permissions or legal authority remain non-transferable.

Both must be represented separately.

## 24. Remote assistance is a dependency path

Remote expert support can improve resilience, but it may depend on:

- communications;
- power;
- compatible diagnostics;
- time zones/availability;
- language/interface;
- data access;
- contractual/authorization arrangements.

It is neither automatically fragile nor automatically available.

## 25. Common-mode repair failures

Multiple assets may share one repair bottleneck:

- same spare part;
- same diagnostic device;
- same technician;
- same vendor;
- same firmware service;
- same calibration laboratory;
- same transport route;
- same workshop power supply.

REGEN-044 must expose this so N assets do not become N independent recovery paths by counting hardware alone.

## 26. Repair queues are explicit

If several failures require the same limited repair resource, order and queueing matter.

The campaign should be able to represent:

```text
repair resource capacity < simultaneous repair demand
```

without pretending all repairs occur at once.

## 27. Priority is policy, not physics

A deterministic model may evaluate a declared repair priority policy, but it must not invent moral or governance priority.

Different adopted policies can be compared as scenarios.

## 28. Safety-critical isolation is preserved

A failed hard-safety condition cannot be bypassed merely to restore service faster.

Where an authoritative safety profile requires isolation, inspection or verification, that remains a hard gate.

## 29. Ecology and repair interact but remain separate

A repair strategy may consume materials, energy or environmental capacity, but ecological obligations remain separately authoritative.

REGEN-044 does not permit service urgency to silently waive REGEN-025 obligations.

## 30. Repair outcome taxonomy

A repair/recovery evaluation should preserve outcomes such as:

```text
RecoverableNow
RecoverableAfterDelay
RecoverableWithSubstitution
BlockedByPart
BlockedByTool
BlockedBySkill
BlockedByAuthority
BlockedByVerification
BlockedByAccess
BlockedByDependency
UnresolvedDiagnosis
NotRepairableUnderDeclaredContext
```

Exact names may change during implementation, but failure reasons must remain distinguishable.

## 31. No universal repairability score

The output should be plural and inspectable.

Potential dimensions include:

- recoverability state;
- time to recover;
- service contribution after recovery;
- spare/part dependency;
- tooling dependency;
- skill concentration;
- authority dependency;
- documentation/software dependency;
- verification dependency;
- common-mode dependency;
- evidence completeness.

The core assigns no universal weights.

## 32. Terra-Preta / regenerative-loop boundary

A local soil-amendment loop may appear material-local while depending on fragile repair chains for shredders, kilns/reactors, pumps, mixers, moisture measurement, weighing, storage handling, transport, laboratory equipment or irrigation systems.

REGEN-044 makes those dependencies visible without prescribing how to operate or repair hazardous equipment.

## 33. Safe abstraction boundary

This contract defines repair dependency evidence and state transitions only.

It deliberately does not provide step-by-step repair instructions, bypass procedures, hazardous equipment modifications, electrical work instructions, pressure-system procedures, combustion procedures, chemical handling recipes or medical-device maintenance procedures.

Operational repair guidance belongs to authoritative manuals, qualified personnel and domain-specific safety systems.

## 34. Symthaea boundary

Symthaea may:

- identify likely repair bottlenecks;
- model repair queues;
- compare spare/skill investments;
- propose training priorities;
- detect common-mode dependencies;
- estimate recovery consequences with uncertainty.

Symthaea may not:

- certify a technician;
- waive safety/authority requirements;
- declare an asset repaired;
- return an asset to service;
- issue physical repair commands.

## 35. Mycelix boundary

Mycelix may coordinate identity, evidence, inventories, qualifications, role/authority references, commitments, documentation references and service state through the domains that own them.

REGEN-044 does not create a new universal maintenance authority.

## 36. Initial deterministic model direction

A later dependency-light evaluator may define structures conceptually equivalent to:

```text
RepairSubjectRef
FailureStateRef
RepairCapabilityRequirement
PartRequirement
ToolRequirement
SkillRequirement
DocumentationRequirement
VerificationRequirement
RepairResourceState
RepairCandidate
RepairEvaluation
RecoveryOutcome
```

with exact references into authoritative domain state.

The pure evaluator should not import Holochain, networking, Symthaea, device-control runtimes or hazardous-operation logic.

## 37. Initial evaluation order

A first deterministic evaluator should use a stable sequence such as:

```text
subject/configuration identity
-> failure/diagnosis state
-> safety isolation state
-> authority eligibility
-> parts compatibility/availability
-> tools/test equipment
-> skill availability
-> documentation/software
-> access/environment
-> repair queue/resource contention
-> verification/return-to-service requirements
-> timing vs service horizon
-> recovery outcome
```

No later soft benefit can erase an earlier hard failure.

## 38. Minimum synthetic campaign

A first executable campaign should include at least:

1. fully supplied/qualified repair candidate becomes recoverable;
2. wrong part revision fails compatibility;
3. zero stock blocks recovery;
4. reserved spare cannot be double-spent;
5. one spare serving two failed assets creates contention;
6. missing tool blocks recovery;
7. unavailable calibration equipment blocks return to service;
8. skilled worker unavailable within horizon blocks continuity recovery;
9. skill exists but authority is expired/unresolved;
10. documentation unavailable blocks a procedure that requires it;
11. cloud-only software dependency fails under communications outage;
12. offline artifact allows that dependency to remain available where otherwise qualified;
13. repair completed but verification pending does not become recovered;
14. recovery after service deadline does not retroactively preserve continuity;
15. cannibalization recovers one asset while explicitly degrading another;
16. local fabrication without qualification remains unverified;
17. multiple assets sharing one technician expose common-mode concentration;
18. repair queue preserves sequencing/resource limits;
19. changing repair priority changes scenario outcome without becoming universal policy;
20. remote assistance requires communications and expert availability;
21. training pipeline does not count trainees as current independent technicians;
22. one instructor remains a concentration risk;
23. stale inventory state forces re-evaluation;
24. stale qualification/authority state forces re-evaluation;
25. maintenance overdue remains visible without invented failure probability;
26. Symthaea confidence cannot manufacture repair qualification;
27. service urgency cannot waive a hard safety gate;
28. successful repair does not automatically prove full service restoration.

## 39. Relationship to REGEN-043

REGEN-044 provides exact repair/skill capability state to substitution analysis.

A substitution depending on a failed/unrepairable asset is not qualified merely because its nominal capacity exists.

## 40. Relationship to REGEN-045

REGEN-045 should consume actual service and recovery trajectories rather than assume every repair candidate succeeds.

```text
repair candidate
!= recovery event
!= continuity outcome
```

## 41. Relationship to REGEN-047

REGEN-047 should adversarially attack repair resilience for shared technicians, shared spares, shared software/vendor dependencies, calibration bottlenecks, queue saturation and stale skill/authority evidence.

## 42. Deliberate non-claims

REGEN-044 creates no current claim of:

- safety of any real repair;
- qualification of any real technician;
- compatibility of any real replacement part;
- adequacy of any real spare inventory;
- repairability of any real hazardous system;
- manufacturer approval;
- legal repair right;
- return-to-service authorization;
- disaster readiness;
- service continuity;
- physical execution.

Its proposition is deliberately narrow:

> regenerative resilience requires repair and skill dependencies to be represented as explicit, stateful, capacity-limited and authority-bounded recovery paths rather than assumed from equipment ownership, spare inventory or nominal local expertise.
