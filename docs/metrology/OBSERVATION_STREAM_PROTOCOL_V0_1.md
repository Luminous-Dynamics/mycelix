# Observation stream identity and timebase v0.1

Parent: [MYC-BUILD-STREAM-TIMEBASE-001 #6054](https://github.com/Luminous-Dynamics/mycelix/issues/6054).

## Purpose

A sample-level passport does not prove that a time series is well formed. This profile makes stream identity, boot/clock epochs, sequence scope, event/acquisition/receipt order, duplicate replay and gaps explicit. It is a deterministic stream conformance contract, not a clock-synchronization service.

## Core identity

A stream is scoped by exact instrument, channel, channel profile, firmware/configuration digests, stream ID, clock domain, boot epoch, timestamp basis and clock origin. Sequence numbers are monotonic only inside that scope. Reboot, reset or wrap opens a new epoch; sequence namespaces are never silently reused.

## Time semantics

- `event_time` is the physical/domain event time in the stream's declared basis.
- `acquisition_time` is when the instrument acquired the sample, in the same basis.
- `collector_receipt_time_unix` is the collector's wall-clock receipt time; it is not substituted for instrument time.
- `display_timezone` is presentation metadata, never the ordering authority.
- Monotonic seconds are only comparable within their exact clock domain and boot epoch.
- A clock-domain conversion requires a versioned mapping, named source and target domains, validity interval, offset/drift model, uncertainty and a verification receipt.

## Replay and coverage

Exact duplicate sample ID/sequence/digest is idempotent. Conflicting reuse is rejected. Out-of-order delivery is preserved and reconciled only within a declared bounded window. Sequence gaps remain gaps; no zero fill or interpolation is permitted. Conflicts, unknown clocks and invalid epochs block downstream aggregation.

## Owner boundaries

Mycelix owns instrument/sample stream identity and timebase qualification. Symtropy #1849 owns domain availability events (`restored`/`lost`) whose logical ticks are not instrument clock readings. Symtropy #4507 consumes qualified trajectories. Sol-Atlas #633/#700 consumes owner-native observations and does not define a competing clock service.

## Claim ceiling

The corpus is entirely synthetic. It does not prove the accuracy of a physical clock, network synchronization, sensor measurement or service outcome.
