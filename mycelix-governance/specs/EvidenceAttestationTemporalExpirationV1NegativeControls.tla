---------------- MODULE EvidenceAttestationTemporalExpirationV1NegativeControls ----------------
EXTENDS EvidenceAttestationTemporalExpirationV1

CONSTANT Control

BadExpiryPersistence ==
  /\ Control = "expiry-persistence"
  /\ now = T1
  /\ GrantExpiry[G1] = T1
  /\ G1 \in activeGrants
  /\ G1 \notin revokedGrants
  /\ now' = T2
  /\ activeGrants' = activeGrants
  /\ revokedGrants' = revokedGrants
  /\ authority' = authority
  /\ effectiveAuthority' = effectiveAuthority
  /\ evidenceRecorded' = evidenceRecorded

NegativeNext == BadExpiryPersistence

NegativeSpec == Init /\ [][NegativeNext]_vars
=========================================================================
