# ROS-005 Typed Scope Profile v1

**Status:** draft Rust reference profile, stacked on ROS-005 parent-chain PR #3679  
**Owner:** Mycelix #6302  
**Consumer:** Xenia #517 / M1 lane mapping in Xenia PR #509

## Scope and compatibility

This profile is additive. It does not alter ROS-005 exact-name v1/v2 ActionScope or ResourceScope semantics. Existing exact-name checks remain equality-only. The new types live in `mycelix_core_types::delegation_scope` and are not yet wired into `Delegation`, Holochain validation, or an execution path.

The profile makes only the following finite dimensions structural:

- finite exact action identifiers;
- finite exact resource identifiers;
- finite relying-party/audience identifiers;
- finite data/control directions;
- exact commitments to owner-native ROS-004 consent, purpose and data-class profiles;
- either any operation within the profile or one exact immutable operation digest;
- static upper bounds for duration, total operations, bytes, concurrency, target count and rate.

This v1 profile deliberately does **not** infer resource hierarchy from path prefixes. For example, `host/a` does not contain `host/ab`. A future hierarchy requires a separately versioned resource type and explicit ancestry proof.

## Containment semantics

A child is no wider only when every dimension is contained:

1. Child action, resource, audience and direction sets are subsets of the parent's finite sets.
2. Consent profile, purpose and data-class commitments match exactly. ROS-004 remains the owner of consent semantics; these commitments do not create consent.
3. An `AnyWithinProfile` operation binding may narrow to an exact digest. An exact digest can only remain the same exact digest.
4. A child may add a numeric bound where the parent is unbounded, or lower an existing maximum. It cannot remove a parent bound or increase it.
5. A child must be a proper subset in at least one modeled dimension when strict attenuation is required. Equal-scope children are rejected.

Incomparable scopes and unknown dimensions are not guessed into a permit. A consumer that cannot verify a required dimension must hold/return an indeterminate result.

### Rate-limit containment

Rate limits use a rolling-window definition: at most `max_events` admitted operations in any interval of `window_ms`. Comparing only event counts or only nominal rates is unsound.

The profile uses a conservative sufficient condition for trace containment. If the child's window is at least as long as the parent's, the child event maximum must not exceed the parent maximum. If the child's window is shorter, partition a parent window into `ceil(parent_window / child_window)` child windows; that multiplied worst-case child count must not exceed the parent maximum. Arithmetic overflow rejects containment. This can reject some safe but difficult-to-prove combinations; it must not accept a known wider trace.

## Canonical identity encoding

Canonical bytes use:

- domain: `mycelix.ros005.typed-scope.v1\0`;
- schema version: unsigned 16-bit big-endian value `1`;
- finite collections sorted lexicographically, with unsigned 32-bit big-endian counts and byte lengths;
- direction enum tags in declaration order;
- exact 32-byte ROS-004 consent/purpose/data-class commitments;
- explicit operation-binding and optional-budget tags;
- unsigned big-endian numeric budget fields.

Identifier collections reject empties, duplicates, ASCII whitespace/control characters, non-ASCII strings, values longer than 256 bytes, and more than 1,024 entries. The constructor normalizes ordering before encoding. The Rust module includes a fixed 162-byte known-answer vector and field-mutation digest test.

The canonical encoding is a content-binding primitive only. It does not authenticate the issuing principal, establish the authority epoch, prove record currentness, or validate a parent chain.

## Budget and time boundary

The budget model is static: it constrains the maximum scope that may be issued. It does not reserve or consume operations/bytes, enforce concurrency, provide crash recovery, or prevent two sibling children from independently spending the same parent allowance. Those require a durable owner-authorized reservation/consumption contract and concurrency qualification; they are not implied by these data types.

`max_duration_ms` and `window_ms` are durations in milliseconds, not absolute timestamps. Existing ROS-005 `i64` grant/request/revocation times still lack an explicitly documented epoch/unit. This profile does not reinterpret those fields or accept caller-selected time/authority epoch as authenticated evidence.

## Required next qualification

- Independent canonical-byte vectors in at least one non-Rust implementation.
- Differential tests for action/resource/audience/direction containment and operation binding.
- Exhaustive finite-set containment and adversarial rate-window trace generation.
- Property tests for component-wise budget bounds, overflow and incomparable scope rejection.
- Parent-chain replay against exact addressable records using the v1 typed scope, not caller-supplied containment booleans.
- Separate durable budget reservation/consumption protocol with concurrent sibling and crash/restart histories.
- Authenticated authority epoch/currentness and delayed-DHT tests owned by Mycelix #4903/#6236.
- Holochain 0.7.1 substrate qualification remains separately owned by #5719.

## Explicit non-claims

This is not a production authorization service, Holochain capability grant, consent engine, revocation propagation service, currentness oracle, durable budget meter, or effect-boundary enforcement mechanism. It does not yet connect to the existing ROS-005 Delegation API or Xenia runtime. The exact-name v1/v2 profile remains unchanged until an owner-approved migration is completed.
