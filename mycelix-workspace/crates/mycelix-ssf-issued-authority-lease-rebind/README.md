# SSF Issued Authority Lease Rebind v0.1

This crate converges direct durable lease issuance and historical restart reconciliation onto the exact typed bounded lease-candidate lineage before any use-time effect-admission boundary.

Historical `Issued` evidence alone is not semantic authority, and a rehydrated candidate alone cannot claim durable issuance. Rebind requires exact equality across the complete issuance subject: reservation binding, candidate ID/nonce, operation, source/target state, scope, consequence, and lease lifetime.

The rebound token restores only the semantic lease lineage. It still requires use-time revalidation, cannot enter effect admission without that revalidation, forbids delegation, and contains no direct state-install or effect capability.
