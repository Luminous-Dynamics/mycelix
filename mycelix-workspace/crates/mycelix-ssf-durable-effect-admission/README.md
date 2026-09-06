# SSF Durable Effect Admission v0.1

This crate records a crash-safe durable **possible-effect frontier** for one exact execution-surface admission candidate. It does not mint a move-only execution capability and never calls an actuator.

The persistence boundary uses a sealed view trait implemented only for the exact `ExecutionSurfaceAdmissionCandidateV1`, so alternate external types cannot bypass the typestate chain while downstream persistence code avoids repeating the full generic parameter list.

The store must enforce both attempt-ID uniqueness and exact admission-subject uniqueness across attempt IDs. A fresh attempt ID cannot durably admit the same execution-admission subject twice.

After the external write begins, store errors and unverifiable postconditions become `OutcomeUnknown`; they never become blind retry permission. `ProvenNotAdmitted` is a final non-persistence statement for that attempt ID.

Successful admission requires the durable frontier to advance exactly one generation to the exact admission record. The resulting token only records that a possible effect lineage has been durably admitted. It still requires a later single-use execution-capability boundary and contains no direct state-install or effect authority.
