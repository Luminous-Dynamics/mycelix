# SSF Capability-Time Effect Revalidation v0.1

This crate revalidates one exact rebound durable possible-effect lineage immediately before any source-owned operation materialization or move-only capability boundary.

The currently authenticated execution generation must equal the exact generation originally admitted: session identity, authenticated transcript, transport authority, actuator policy, actuator health, operation carrier, and surface identity all remain generation-significant.

A fresh uncertainty-aware `QualifiedCurrentTimeV1` is required. The final capability-readiness ceiling is the minimum of the original effect-eligibility ceiling, current verifier generation, live execution generation, revalidation receipt, and trusted-time receipt. Fresh evidence never resurrects stale authority.

The resulting token still contains no effect capability. It explicitly requires source-owned immutable operation material before capability issuance.
