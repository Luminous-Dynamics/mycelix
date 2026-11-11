# Building measurement stream timebase v0.1

This is the first executable contract slice for Mycelix #6054. It composes with the per-observation passport but does not replace it. The unit under test is a versioned observation stream, not a timestamp/value row.

## What is qualified

- stream identity: instrument, channel, profile, firmware/protocol digests, clock domain, boot epoch and sequence scope;
- timestamp basis, unit and origin, with event time, acquisition time and collector receipt time stored separately;
- monotonic sequence continuity within a declared boot epoch, idempotent duplicate packets, conflicting duplicate rejection and declared modulo wrap;
- bounded out-of-order transport, sequence gaps and completeness against an expected sample count;
- clock-source, reboot and counter-reset transitions;
- clock-correction mapping identity, source/target domains, validity interval, uncertainty, residual and source receipt;
- local-time/DST/leap-second ambiguity, explicit precision loss and the non-authority of collector receipt time for physical rates/integrals;
- targeted invalidation assertions that preserve raw observations and unrelated streams, while #6042 remains the owner of the dependency graph itself.

## Claim ceiling

The highest positive result is `REVIEW_ELIGIBLE_ONLY`. This is synthetic stream identity/order/replay conformance. It does not prove an actual instrument clock's accuracy, network reliability, physical measurement correctness, or the correctness of a derived energy/thermal/flow value. Qualified sample streams are consumed by Symtropy #4507 and Sol-Atlas #633/#700; those owners retain their own semantics.

## Important boundaries

- A monotonic tick is comparable only within its clock domain and boot epoch unless a verified, time-window-applicable mapping exists.
- Local timezone is presentation context, not the time-ordering authority.
- A sequence gap is unknown missingness, never zero and never automatic interpolation.
- Identical duplicate packet identity/digest is idempotent; the same stream/epoch/sequence with conflicting content is a conflict.
- A clock mapping change invalidates affected time-dependent descendants through Mycelix #6042; it does not rewrite immutable raw samples or invalidate unrelated room streams.

## Synthetic corpus

40 frozen cases exercise duplicates/conflicts, gaps, reorder windows, reboot/wrap, clock steps, NTP/PTP changes, synchronization limits, units/origins, local-time/DST/leap seconds, receipt/event/acquisition semantics, firmware/serial changes, mapping correction and targeted invalidation.
