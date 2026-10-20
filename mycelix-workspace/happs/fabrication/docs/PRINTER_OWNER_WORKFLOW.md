# Local-First 3D Printer Owner Workflow: From Intent to Verified Part

**Status:** Research and integration contract. It is not a supported product release, a universal printer compatibility claim, or permission for autonomous machine control.

**Tracking:** [FAB-UX-001 #5041](https://github.com/Luminous-Dynamics/mycelix/issues/5041)

## Executive direction

Make Mycelix + Symthaea + the Luminous Platform the best *owner-controlled* way to use a printer by solving the whole job lifecycle—not by replacing every CAD tool, slicer and firmware stack at once.

The user should be able to say what they want to make, identify which machine and material they own, understand the trade-offs, verify a proposed print, explicitly authorize execution, follow a useful live status, inspect the physical result, and reuse the lessons next time. It should work locally with no mandatory cloud account and preserve export/migration paths.

The product promise is **less guesswork, safer execution, useful repairability, and evidence that improves over time**. It is not “AI-generated geometry is always printable” and not “a sensor anomaly proves that a part is defective.”

## Why the timing is right

- The [3MF format](https://3mf.io/spec/) is the preferred rich-project interchange when supported; ISO/IEC 25422:2025 specifies the 3MF format suite ([ISO catalogue](https://www.iso.org/standard/90283.html)). It can carry more project context than STL alone, but it does not create universal compatibility for printer-control APIs.
- [OrcaSlicer](https://github.com/OrcaSlicer/OrcaSlicer) documents a CLI for headless load/transform/slice/export operations; treat a selected existing slicer as an execution tool instead of immediately attempting a competing slicer implementation ([CLI documentation](https://github.com/OrcaSlicer/OrcaSlicer/wiki/cli_mode)).
- [Moonraker](https://moonraker.readthedocs.io/en/latest/external_api/introduction/) provides HTTP/WebSocket interfaces for Klipper-hosted printers. [OctoPrint](https://docs.octoprint.org/en/main/api/index.html) exposes documented endpoints for file, job, printer and slicing operations. These are adapter targets, not one interchangeable protocol.
- OctoPrint's [authorization documentation](https://docs.octoprint.org/en/main/api/general.html#authorization) explicitly warns that with access control disabled, unauthenticated calls may be treated as having full administrator rights. The product must make local networking and least-privilege setup part of onboarding, not an advanced afterthought.

## Canonical ownership: compose, do not duplicate

| Responsibility | Canonical owner | Product contribution |
|---|---|---|
| Process identity, capability requirements, recipes and process reasoning | Symthaea MFG-PROC-000 ([#5686](https://github.com/Luminous-Dynamics/symthaea/issues/5686)) | Turn user intent into reviewable process/profile requirements |
| Distributed process/resource registry, routing, work orders and lineage | Mycelix MFG-PROC-001 ([#3085](https://github.com/Luminous-Dynamics/mycelix/issues/3085)) and its existing child work | Connect exact process/profile references to printer/resource offers and job records |
| Machine-program validation, session authority, execution and receipts | Symthaea fabrication kernel | Validate before execution and bind command authority to the exact prepared artifact |
| Sensor observations and deterministic anomaly detection | Mycelix Fabrication Process Monitoring (#4283; [implementation PR #4286](https://github.com/Luminous-Dynamics/mycelix/pull/4286)) | Show contextual, source-bound alerts; don't reimplement its detector |
| PrintJob identity, authorization and lifecycle | Existing Fabrication Prints coordinator/integrity work, including [#4256](https://github.com/Luminous-Dynamics/mycelix/pull/4256) and [#4267](https://github.com/Luminous-Dynamics/mycelix/pull/4267) | Use existing authorized state transitions, never bypass them from UI |
| Hardware identities, release/provenance and design artifacts | Mycelix Hardware MHW work ([#2381](https://github.com/Luminous-Dynamics/mycelix/pull/2381), [#2399](https://github.com/Luminous-Dynamics/mycelix/pull/2399), [#2400](https://github.com/Luminous-Dynamics/mycelix/pull/2400)) | Reuse exact design, part, licence and evidence references |
| User interface, device secret handling, local deployment | Luminous Platform / approved system-security owners | Provide local-first setup, operation review, notifications, export and recovery |

Do not add a new Printer, PrintJob, Observation, ProcessDefinition or evidence/provenance root for this user experience. Before implementation, audit the exact qualified owner successor and schema branch; open/draft PRs are not automatically qualified dependencies.

## Proposed end-to-end owner journey

### 1. Discover and connect

- Discover a printer only by a user-approved local endpoint or explicit manual entry; show firmware, adapter/version and verified capability inventory.
- Start with a read-only probe. Do not heat, move, configure, upload or start a job just to identify a printer.
- Detect capabilities rather than infer them from a marketing name. Missing nozzle, bed, toolhead, material sensor, enclosure, probe or firmware features remain unknown.
- Keep device URLs, credentials and local topology local by default. No printer API should be exposed directly to the public internet as a quick-connect shortcut.
- Provide a diagnostic status in ordinary language: connected/read-only, supported for preparation, execution-capable with declared limits, unsupported, stale capability data, or disconnected.

### 2. Express the part intent

The owner can begin from:
- an existing model with its source/licence;
- a parameterized design from an established CAD tool;
- a geometry revision suggested by the assistant;
- a replacement part tied to a verified parent article/revision.

The intent should cover function, envelope/dimensions, fit and interfaces, intended material and environment, load/temperature/chemical exposure where relevant, aesthetic/finish priorities, and cost/time constraints. Ask only for constraints that alter the design or process decision; expose unknowns.

Any AI-generated geometry or repair suggestion is an editable candidate. Preserve original source, requested constraints, generated changes and reviewer decisions. Do not claim fatigue, pressure, heat, electrical, food-contact, medical or other fitness solely because a mesh is watertight or a print succeeds.

For parts used in the computer-repair/productive-closure programme ([Symthaea #5796](https://github.com/Luminous-Dynamics/symthaea/issues/5796), with a first measurable memory-efficiency task in [#7515](https://github.com/Luminous-Dynamics/symthaea/issues/7515)), prioritize non-critical, inspectable enclosures, cable guides, fans/ducts and fixtures where appropriate. Do not treat a printed part as an automatically qualified substitute for semiconductor, mains-voltage, fire-safety, pressure-containing or load-critical components.

### 3. Prepare a portable project

- Prefer 3MF when the source and selected tools support it; preserve STEP/native CAD/source files where they matter. Don't silently collapse parametric source to a lossy mesh.
- Freeze the exact project digest, object transforms, dimensions, units, support decisions, selected slicer/version, machine profile, material profile, nozzle/toolhead, firmware/configuration identity, and any post-processing.
- Resolve printer/material/process capability references from canonical Symthaea profiles. A community profile is a versioned proposal until its evidence and scope are established.
- Run the selected slicer through a pinned CLI/service adapter. Return the complete effective settings and generated artifact, not only the UI form that was submitted.
- Track import/export loss and unknown fields. Unsupported extension metadata is not invented or silently mapped into a known capability.

### 4. Validate and preview before the printer can move

Provide a pre-flight report, with each check marked satisfied, unsatisfied, unknown or not applicable:
- model bounds, scale/units, mesh validity and build-volume fit;
- orientation, supports, tool clearance and known collision/boundary checks supported by the selected tools;
- material/nozzle/process compatibility and temperature/speed ranges against the exact declared profile;
- required firmware features and all machine-specific commands in generated G-code;
- bed/tool count, material availability, estimated time/material and dependencies;
- calibration/profile freshness and conflicts with the current printer configuration;
- licences, attribution and source provenance;
- risk categories, missing checks and conditions that need human inspection.

A checker may reject a known-invalid program; it must not claim to prove every physical failure is impossible. Any change to model, G-code, printer identity, profile, firmware configuration or material after approval invalidates that approval.

The preview should expose the actual prepared toolpath, settings diff, estimated trade-offs and unresolved warnings. Estimates must identify their source and uncertainty. A green visual treatment must never imply “certified” or “guaranteed.”

### 5. Require explicit authorization for execution

- Bind the start request to the exact G-code/artifact digest, printer identity, material/process/profile revisions and current capability snapshot.
- Separate permissions for read status, upload artifact, select artifact, start, pause/resume, cancel, change configuration and send arbitrary commands.
- Confirm the exact part/printer/job with the authorized owner after preview; approval cannot be replayed after any bound input changes.
- Use the existing PrintJob lifecycle and authorization predicate; reject illegal jumps, terminal-state re-entry and identity/configuration replacement.
- Keep emergency-stop/pause routes visible and directly accessible. Preserve hardware/firmware interlocks. A cloud or assistant failure must not disable native safety functions.
- Initial version can recommend a job and execute only after direct owner confirmation. It must not autonomously tune temperature, motion, flow or machine safety limits.

### 6. Monitor the process honestly

Re-use the current FPM owner for the sensor channels and baseline semantics it supports. Display:
- measured value + unit + sensor/channel identity + age and data-quality status;
- baseline/detector ID, version, committed input provenance and anomaly class;
- separate anomaly score, physical severity, confidence/probability (only if a separately calibrated model exists), and part-quality assessment;
- missing/stale telemetry, calibration changes and sensor disagreement.

A high anomaly score is not a defect probability. A normal sensor trace is not proof of dimensional or mechanical conformance. Do not auto-abort, alter setpoints or suppress an operator alert solely because the first deterministic detector reports an anomaly or no anomaly. Any later closed-loop capability is a separate qualification project with a threat model, failure analysis, simulation, hardware-in-loop validation, overrides and bounded authority.

### 7. Close the loop with the physical result

After job completion, ask the owner to classify the result (accepted for intended use, visual defect, fit failure, dimensional measurement, failed print, unknown) and optionally attach evidence. Capture exact artifact/profile/material/firmware/machine revision, timestamps with declared clock source, process traces, observed deviations, rework and inspection method.

Keep:
- print lifecycle completion separate from output acceptance;
- owner self-report separate from independent dimensional/metrology test;
- measurements separate from derived estimates;
- synthetic/simulated evidence separate from physical observations;
- a local file hash separate from truth of the measured claim.

Use this evidence to improve a profile only within its declared printer revision, toolhead/nozzle, material batch/family, environment, calibration state and process envelope. Don't train one global success rate from incomparable jobs.

### 8. Turn the community into a repair and learning commons

A useful maker network should support:
- openly licensed parts and derivative forks with explicit obligations;
- shareable machine/material profiles with compatibility and scope;
- repair guides, common-failure diagnosis and replacement-part links;
- small local job offers and service-provider capability records;
- price/time/waste estimates that separate observed from predicted values;
- verified substitute candidates and test receipts;
- versioned retractions when a profile or part is found defective.

Mycelix can provide identity, permissions, resource offers and production/evidence lineage; it does not convert community votes or popularity into engineering qualification. Users must be able to export jobs/profiles and keep private models, customer data, serial numbers and exact household topology local.

## Architecture and security boundary

Use thin protocol adapters over canonical process/evidence semantics. Start with:
1. Moonraker/Klipper read-only discovery and status;
2. a selected slicer CLI for deterministic preparation, with a frozen project fixture;
3. OctoPrint adapter after shared authorization/transition semantics are clear;
4. explicit execution confirmation and receipt;
5. FPM monitoring plus a separate post-print assessment flow.

Do not start with a generic plugin bus, proprietary cloud, new slicer, or autonomous closed-loop control. Adapter support must be proved per capability and version. Map vendor-specific values loss-aware into canonical owners; preserve unknowns.

Secrets belong in approved local secret storage, never in print artefacts, Git, public Mycelix entries or diagnostic logs. Validate origins and certificates when network mode is used; require authentication; minimize outbound traffic; rate-limit mutation commands; and offer offline mode. Security tests must cover credential leakage, stale approval replay, unauthorized start/stop/config edits, forged capabilities, wrong-printer/artifact binding and network loss during state change.

## Read-only adapter conformance fixture

A draft, machine-readable failure/adversarial fixture corpus is checked in at [`docs/fixtures/printer-read-only-conformance-v1.json`](fixtures/printer-read-only-conformance-v1.json). It defines 11 cases for read-only Moonraker discovery/status, including healthy/not-ready/degraded server states, 401 authentication failure, malformed/missing data, stale status, cross-origin redirects, unknown capabilities, and attempted state-changing HTTP/JSON-RPC operations.

The corpus explicitly requires mutation attempts to be denied **before the network request is sent** during P1, credentials never to reach unrelated redirects or logs, and missing/stale values to remain unknown rather than healthy/zero. The fixture file is a contract for future tests, not executable evidence that an adapter already complies. No live printer was contacted, and no runtime code/test was added in this PR.

## Acceptance gates

| Gate | Must demonstrate | Explicit non-claim |
|---|---|---|
| P0: ownership/threat model | Existing owner/type/API map, data-flow map, authority matrix and adversarial cases reviewed | No implementation or compatibility implied |
| P1: read-only connect | Local discovery/status works for one declared Moonraker/Klipper configuration without changing machine state; unknowns remain unknown | No job upload or print start |
| P2: deterministic prepare | Frozen 3MF/model fixture, pinned slicer build, exact config inputs, repeatable artifact/receipt digests and static rejection corpus | No physical quality or general safety claim |
| P3: owner-authorized print | Exact artifact/profile/printer binding, explicit confirmation, valid existing lifecycle transitions, audit receipt and successful revoke/unauthorized controls | No unattended/autonomous printing claim |
| P4: truthful monitoring | Frozen synthetic FPM corpus passes; consented real traces and independent inspection are separately evaluated | No calibrated defect probability without empirical validation |
| P5: physical outcome | Defined dimensional/functional inspection and replicate plan; failed/unknown results retained | No universal process qualification |
| P6: multi-vendor extension | Additional adapter passes the same conformance suite with exact version/capability scope and documented losses | No claim of universal printer compatibility |

## Recommended success metrics

Measure what makes life better for the owner:
- time from model choice to a *reviewable, correct preparation*;
- first-print success and reprint rate, with denominator, job class and outcome source declared;
- owner intervention and setup/calibration burden;
- material waste, support waste, energy and completion time with method and uncertainty;
- rate of prevented unsafe/invalid submissions, plus false rejection burden;
- anomaly alert precision/recall only where independent labels exist;
- measured physical-fit outcome on a named test corpus;
- ability to move/export a project, slicer settings and result receipt to another supported host;
- repair time/cost and verified availability of a compatible replacement part.

No headline number should combine visual appearance, fit, strength, safety, sensor anomaly or job completion into one unqualified “print quality” score.

## Immediate next execution

1. Review/merge FPM and print-job security changes only on their own exact, qualified heads; do not assume a linked open PR is already a usable dependency.
2. Build a read-only Moonraker inventory probe and contract fixtures first; include unreachable printer, unauthorized API, stale firmware/profile, unknown toolhead capability and network-drop controls.
3. Freeze a tiny 3MF + selected-slicer fixture set and repeat the same pinned CLI preparation twice, comparing settings and output receipts.
4. Exercise the complete preview-to-confirm transition against the existing PrintJob lifecycle with no arbitrary G-code command path.
5. Add real printer monitoring only after a physical trace's source, clock, units, calibration and owner consent are recorded; compare alarms with independent visual/dimensional inspections.

This is the end-to-end product specification requested by #5041. It adds no production type and asserts no working cross-vendor UI, no universal compatibility, no qualified printer control, and no physical quality result.
