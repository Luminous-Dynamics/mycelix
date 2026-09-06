# SSF Effect Admission Rebind v0.1

This crate converges direct durable effect admission and historical restart reconciliation onto the exact typed execution-surface admission candidate before any capability-time execution revalidation or move-only effect capability boundary.

Historical `Admitted` evidence alone is insufficient to recover the typed execution lineage, and a rehydrated execution-admission candidate alone cannot claim durable effect admission. Rebind requires exact equality of the complete durable admission subject.

The resulting token restores only the possible-effect lineage. Both direct and historical paths still require a fresh capability-time execution revalidation because time, transport/session generation, actuator policy, and actuator health may have changed after durable admission.

It contains no single-use effect capability and cannot call an actuator.
