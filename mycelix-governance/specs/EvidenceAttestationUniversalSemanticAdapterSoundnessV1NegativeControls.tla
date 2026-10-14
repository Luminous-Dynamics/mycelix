---------------- MODULE EvidenceAttestationUniversalSemanticAdapterSoundnessV1NegativeControls ----------------
EXTENDS EvidenceAttestationUniversalSemanticAdapterSoundnessV1
CONSTANT Control
NegativeCommit == /\ Control # "" /\ committed' = TRUE
NegativeNext == NegativeCommit
NegativeSpec == Init /\ [][NegativeNext]_vars
=============================================================================