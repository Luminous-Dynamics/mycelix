# SSF Historical Actuator Invocation Activation v0.1

Activates an already-journaled initial or replay actuator attempt after same-identity historical journal reconciliation.

Historical reconciliation proves what happened to the old journal under a newer trusted journal-store policy/key generation. This crate asks a different question: is that exact old attempt still eligible to approach the actuator now?

Fresh trusted time must not precede either the original attempt time or the historical reconciliation observation. The activation ceiling is the minimum of the historical invocation-eligibility ceiling and the new trusted-time evidence.

No journal write, actuator reservation, invocation, replay authorization, or external effect occurs here.