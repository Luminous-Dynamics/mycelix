# Mycelix Experience Qualification v1

Status: **measurement contract**

Scope: cross-domain Mycelix frontend experience. This contract defines what must be measured before UX changes are described as improvements. It does not manufacture a baseline and does not convert subjective review into qualification evidence.

## Governing rule

> **A UX claim is evidence-bearing only when it names the task, subject, method, result, and truth boundary being measured.**

Visual preference is useful design feedback, but it is not by itself evidence of task completion, comprehension, accessibility, or trust calibration.

## Canonical task chain

The first baseline MUST preserve these scenarios as independently recordable tasks:

1. `first_launch` — enter the product from a fresh local state and identify the next useful action.
2. `find` — locate a known person, space, document, project, learning item, conversation, or action.
3. `create` — create a domain object without requiring protocol vocabulary.
4. `share` — share that object with an intended person or group.
5. `understand_visibility` — correctly state who can see the object and what remains local/private.
6. `inspect_provenance` — locate the explanation/evidence surface and correctly distinguish availability from verification.
7. `offline_continue` — lose connectivity, continue an eligible local-first task, and correctly understand what has and has not synchronized.
8. `recover_device` — resume/recover the intended user state on another device through the supported recovery path.

A domain MAY add scenarios. It MUST NOT silently drop a canonical scenario because the current implementation makes the task difficult.

## Required measurements

Each scenario record SHOULD capture, when applicable:

- completion: pass / fail / abandoned;
- time to completion;
- wrong turns or route reversals;
- consequential errors;
- help requests;
- number of protocol-specific terms the participant had to interpret;
- post-task confidence as a subjective measure, clearly separated from correctness;
- truth comprehension questions relevant to the task;
- keyboard-only completion result;
- narrow/mobile completion result;
- offline/degraded completion result where applicable.

A score MAY summarize measurements only if the raw measurements and scoring rule remain inspectable.

## Truth-comprehension probes

A task involving consequential state MUST ask the user to distinguish the relevant states in plain language. Examples:

- Is this item available, or has it actually been verified?
- Is this action authorized, pending, executed, or settled?
- Is this work saved locally, synchronized remotely, or both?
- Does a connected transport mean this action is currently authorized?
- If evidence is stale, is it absent or present-but-old?
- If the UI says Unknown, what fact is currently not established?

A correct click path with an incorrect truth model is NOT a fully successful task.

## Local-first qualification

Offline-first behavior MUST be measured as a continuity property, not as an error-screen property.

For operations the domain permits locally, qualification SHOULD establish that:

1. the user can continue the eligible task while disconnected;
2. locally durable work is not visually downgraded to nonexistence;
3. the UI distinguishes local durability from remote synchronization;
4. reconnect does not silently duplicate a consequential action;
5. conflicts, rejections, or remote divergence remain inspectable;
6. the user can tell whether closing or switching devices risks unsynchronized work.

`Saved locally != synchronized != federated != confirmed != settled`.

## Accessibility qualification

Automated checks are necessary but insufficient.

The qualification set SHOULD include:

- explicit label/control association for forms;
- accessible names for dialogs and consequential controls;
- keyboard-only operation;
- visible focus;
- modal focus entry, containment, Escape behavior, and restoration;
- semantic status/error announcements where appropriate;
- reduced-motion behavior that preserves all consequential information;
- zoom/reflow and narrow viewport checks;
- manual screen-reader smoke testing for high-consequence workflows.

Accessibility regressions that hide or strengthen truth are also frontend truth-invariant regressions.

## Responsive/device matrix

At minimum, record results for:

- narrow phone viewport;
- wide phone/small tablet viewport;
- desktop viewport;
- 200% text zoom for representative flows;
- 400% reflow for representative information surfaces where applicable.

Device class is part of the evidence record. A desktop-only pass MUST NOT be generalized to mobile.

## Evidence record

A UX qualification record SHOULD include:

- `subject_commit` — exact Git commit under test;
- `scenario_id`;
- `domain`;
- `device_class`;
- `input_mode` — pointer / keyboard / touch / assistive technology;
- `network_mode` — online / offline / degraded / reconnecting;
- `participant_class` — e.g. novice / experienced, without storing unnecessary identifying data;
- `started_at` and `completed_at` or derived duration;
- `completion_result`;
- `consequential_errors`;
- `wrong_turns`;
- `truth_probe_answers`;
- `artifact_refs` — screenshots, traces, recordings, test reports, or notes where collected;
- `observer_notes`;
- `tool_versions` for automated evidence.

The exact subject commit MUST be recorded. Results from one commit MUST NOT be silently promoted to a replacement subject.

## Baseline rule

The first run is a baseline, not a pass/fail target unless a requirement already exists.

Improvement claims MUST compare compatible populations, tasks, environments, and metrics. If any materially differ, the report must say so rather than presenting the measurements as directly comparable.

## Proposed release gates

The following are candidate release-blocking conditions once the corresponding harness exists:

- a canonical task becomes impossible by keyboard;
- a modal loses an accessible name or traps/restores focus incorrectly;
- a form error is not programmatically associated with its control;
- a consequential status relies on color/icon/animation alone;
- offline/local work is presented as remotely synchronized without evidence;
- a truth-comprehension probe regresses because presentation strengthens the underlying state;
- narrow/mobile layout makes a canonical task impossible;
- a replacement exact head is claimed qualified using evidence from an older subject.

Numeric performance or usability thresholds MUST be introduced only with an explicit rationale and retained baseline evidence. This document intentionally invents none.

## Recommended automation ladder

1. Rust/unit tests for pure state semantics and helper contracts.
2. wasm compilation and warnings-denied linting.
3. static accessibility checks where deterministic.
4. component rendering tests.
5. browser keyboard/focus tests.
6. automated accessibility scans.
7. responsive viewport tests.
8. offline/reconnect browser tests.
9. visual regression for presentation drift.
10. task-level human usability/comprehension studies.

Passing an earlier rung does not imply passing a later one.

## Review questions

For every UX PR, reviewers SHOULD ask:

1. What user task becomes easier, safer, faster, or clearer?
2. What exact measurement would demonstrate that claim?
3. Which truth invariant could this presentation accidentally strengthen?
4. What happens offline, on a narrow viewport, and with keyboard-only input?
5. Does this change move domain authority into shared presentation code?
6. Does the evidence correspond to this exact commit?
