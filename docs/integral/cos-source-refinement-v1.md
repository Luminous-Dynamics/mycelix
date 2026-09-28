# COS Source Refinement v1

## Purpose
This document binds the COS/ProductiveLoopV1 semantic model to the canonical `mycelix-manufacturing::manufacturing_common` source types.

The adapter is intentionally one-way and claim-bounded: source objects can satisfy only the semantic target their fields actually represent. The adapter must never synthesize a missing physical observation, material-consumption receipt, quality qualification, or temporal availability observation.

## Concrete mappings

| Source-owned object | Permitted refinement | Explicitly rejected shortcut |
|---|---|---|
| `WorkOrder` | production-plan evidence | `Completed`/`Closed` -> observed work |
| `BillOfMaterials` | planned material requirement | BOM -> actual material consumption |
| `MrpResult::scheduled_operations` | scheduled operation | schedule -> observed work |
| `Machine::capabilities` | registry-level capability | capability -> current availability |
| `Machine::status` | registry lifecycle fact | `Available` -> timestamped current availability |
| `WorkOrder` lifecycle | lifecycle state | completion -> qualified output |

## Why the distinctions matter

`WorkOrderStatus::Completed` proves that the source object entered a lifecycle state. It does not, by itself, bind a worker/action/equipment observation to the work order.

A BOM specifies intended material requirements. It does not identify the actual lot, quantity, timestamp, operator/system receipt, or other evidence needed to establish consumption.

MRP can schedule an operation without establishing that the operation was executed. This preserves the `plan != execution` and `declared work != observed work` boundaries.

A registered machine may have a declared capability and an `Available` status, but the current `Machine` type contains no timestamped observation, freshness policy, calibration/maintenance evidence, or temporal validity interval for that status. It therefore cannot by itself satisfy a current-availability requirement.

A completed work order also cannot establish quality or safety qualification. Those require explicit qualification evidence and a profile-defined acceptance binding.

## Formal/conformance links

- `COS-FV-002`: declared/planned work is distinct from observed work.
- `COS-FV-003`: capability is distinct from current availability.
- `COS-FV-004`: useful/completed output is distinct from qualification.
- `COS-FV-005`: temporal freshness must be explicit where currentness is required.
- `COS-FV-007`: plan is distinct from execution.
- `PL-F-001`: planned input is distinct from consumed input.
- `PL-F-002`: declared work is distinct from observed work.
- `PL-F-003`: useful output is distinct from qualified/safe output.
- `PL-F-006`: capability is distinct from availability.

## Qualification status

The source-refinement adapter is **source-refined and executable-testable**, but it is not a production qualification or formal proof of the manufacturing system.

Next closure step is to introduce source-owned observed-work, material-consumption, quality, and availability evidence types (or adapters to existing owners if already present), then bind each concrete transition to those receipts without weakening the negative corpus.

## Nonclaims

This artifact does not establish:

- that manufacturing actually occurred;
- product quality or safety;
- machine uptime or operational readiness;
- material provenance or actual consumption;
- general manufacturing capability from one run;
- economic performance;
- ecological performance;
- Integral/COS external validation;
- N2 or any broader societal/economic outcome.

The strongest current claim is semantic refinement of existing source types with executable adversarial tests.