# SSF Final Effect Capability Revalidation v0.1

This crate performs the final fresh security proof before a transient move-only effect capability may be constructed. It still does not mint that capability or invoke an actuator.

The final proof requires the currently authenticated execution generation to equal the exact originally admitted generation, the currently trusted operation-provider descriptor to equal the provider generation that owns the opaque operation handle, and a verifier receipt to rebind the exact durable capability lineage, source-owned material receipt, live execution generation, provider descriptor, and opaque handle.

Fresh uncertainty-aware trusted time is checked again. The final readiness lifetime is the minimum of the inherited durable capability ceiling, source-owned material lifetime, execution generation, provider generation, final verifier receipt, and trusted-time receipt.

Any session transcript, transport generation, actuator policy/health generation, operation carrier, execution time basis, provider policy/generation, or handle change fails closed in v0.1 and requires a new qualification/admission lineage.
