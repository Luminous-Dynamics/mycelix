# hearth-care-recurrence

Pure recurrence and calendar-intent contract for Hearth Care.

This crate separates household intent from timezone expansion. It defines canonical local calendar values, typed recurrence rules, IANA timezone identity, explicit DST gap/fold policies, exclusions, one-off overrides, missed-occurrence policy, bounded expansion requests, and verifiable expansion receipts.

It deliberately does **not** read the host timezone database or perform timezone conversion. A later adapter may use a pinned timezone/RRULE engine, but DHT evidence binds the engine and tzdb references so consensus never depends on an unversioned `/usr/share/zoneinfo`.

Concrete Care occurrence identity remains `(schedule, resolved UTC window)` in the Care ledger; recurrence instance identity remains `(schedule, original local start)` so reassignment, timezone resolution, and one-off overrides do not accidentally create duplicate conceptual chores.
