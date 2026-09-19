# hearth-care-timezone-jiff

Pinned civil-time resolution adapter for the Hearth Care recurrence contract.

This crate deliberately uses **only** Jiff's bundled time-zone database. It disables Jiff's default system/global timezone features and pins both `jiff` and `jiff-tzdb` exactly. No `/usr/share/zoneinfo`, `TZDIR`, browser timezone, or implicit global lookup is permitted on this path.

The adapter resolves one already-selected local recurrence instance. It does **not** enumerate recurrence rules and therefore cannot become a second recurrence authority. Gap/fold behavior comes from the explicit `CivilTimePolicy` in `hearth-care-recurrence`, and results carry exact engine/tzdb evidence.

Fresh qualification must include native + WASM builds, bundled-tzdb identity, DST gap/fold vectors, non-hour transitions, invalid-zone handling, reproducibility, and binary-size/resource measurements before this adapter can authorize Care materialization.
