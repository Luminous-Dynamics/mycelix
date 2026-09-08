# SSF Canonical Replay Evidence Qualification Request v0.1

Fresh, anti-rollback presentation boundary for the immutable canonical replay-evidence subject.

This layer does not qualify evidence and does not invoke any evidence verifier. It binds the exact frozen subject to a fresh trusted-time observation and first requires the subject's explicit `UnixMillisecondsUtc` time basis before comparing any freshness values.

The request then requires that qualification time does not move backward behind the history-read time and does not exceed the inherited subject validity ceiling.

The resulting request subject binds the exact evidence subject, qualification-time receipt, latest plausible qualification time, and natural-expiry ceiling. There is no replay authority, effect authority, or third-attempt permission.

A future evidence qualifier should accept only this request boundary rather than a naked evidence subject, so `what was considered`, `when it was considered`, and the clock domain used for that decision remain inseparable.
