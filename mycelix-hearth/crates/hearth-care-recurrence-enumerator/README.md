# hearth-care-recurrence-enumerator

Pure local-calendar recurrence enumeration for Hearth Care.

This crate consumes the A6.1 recurrence contract and emits canonical **original local starts only**. It performs no timezone lookup, DST handling, exclusions, overrides, assignment changes, or Care writes.

Schema-v1 enumeration semantics are intentionally fixed and documented:

- multi-week intervals use Monday as the canonical week start (RFC 5545 `WKST` default);
- invalid calendar dates are skipped rather than clamped (for example monthly day 31 skips February);
- `COUNT` counts generated recurrence instances beginning with `starts_local` before exclusions or one-off overrides;
- `UNTIL` is inclusive by local date;
- exclusions and overrides are a later recurrence-set transformation and never extend a COUNT-limited series;
- open-ended local scans are bounded to ten years per call and must be chunked by callers.

This separation keeps rule enumeration deterministic and independently testable from timezone resolution in `hearth-care-timezone-jiff`.
