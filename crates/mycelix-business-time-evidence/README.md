# mycelix-business-time-evidence

Preregistered, transition-aware local-time evidence for Mycelix Business qualification.

The Business Fabric should not hard-code countries, timezone rules, DST calendars, or a particular timezone database into its economic core. It does, however, need deterministic civil-time semantics when qualification depends on dayparts, weekends, schedules, cutoffs, or regulatory windows.

This crate binds an externally supplied rule source and exact UTC-offset schedule before evaluation.

## Contract

`LocalTimeSchedule` binds:

- opaque timezone identity;
- opaque time-rule source identity and digest;
- preregistration time;
- exact evaluation window;
- contiguous UTC periods with explicit offsets; and
- a deterministic schedule digest.

Periods must cover the evaluation window exactly, with no gaps or overlaps. Offsets are bounded to +/-14 hours. Resolution outside the registered window fails closed.

## Transitions

`resolve_window` reports every offset transition inside a UTC interval rather than pretending one fixed offset governed the whole interval. Downstream profiles can therefore reject, split, or explicitly handle a target window crossing a DST/historical transition.

Mapping UTC -> local time is deterministic even when a fall-back transition repeats a local wall-clock hour, because the UTC instant and active offset remain explicit.

## Non-claims

This crate does **not** ship a timezone database and does not establish that a supplied schedule is the legally or historically correct schedule for a named place. A separate adapter/evidence source can bind a particular tzdata or jurisdictional ruleset to `rule_source_digest`.

The schedule is evidence for deterministic interpretation, not business authority.
