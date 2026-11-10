# Building condition applicability oracle v0.1

This package evaluates whether a specific sample was taken under the declared conditions for which the relevant instrument/method claim is valid.

Four synthetic method domains are represented: thermal environment (ISO 7726:2025), closed-conduit water metering (ISO 4064-1:2024 / OIML R 49:2024), AC power quality (IEC 61000-4-30:2025 corrected 2026-07), and PM2.5 measurement (ISO 16000-37:2019). The synthetic numeric envelopes are deliberately illustrative and do not reproduce normative limits.

Numeric results use conservative interval containment: only full uncertainty-interval containment is `WITHIN`; wholly disjoint is `OUTSIDE`; boundary overlap is `INDETERMINATE`. Categorical values require exact membership and a verified-configuration record.

The oracle requires exact method/revision, profile revision, timebase, clock epoch, validity window, spatial scope, independently admitted source, current calibration where numeric reference instrumentation is used, valid QC and authorized rights. A subject sensor cannot certify its own conditions.

Claim ceiling: synthetic profile/observation applicability only. It does not qualify real instruments, methods, standards conformity, electrical safety, water/air quality, thermal comfort or building performance.
