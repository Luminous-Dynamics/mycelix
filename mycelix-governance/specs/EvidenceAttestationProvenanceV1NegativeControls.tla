---------------- MODULE EvidenceAttestationProvenanceV1NegativeControls ----------------
EXTENDS EvidenceAttestationProvenanceV1

CONSTANT Control

BadUntrustedAttestation ==
  / Control = "untrusted-attestation"
  / E1 
otin evidenceRecorded
  / SignatureValid[E1]
  / ~SignerTrusted[E1]
  / EvidenceSubject[E1] = TargetSubject[E1]
  / G2 
otin activeGrants
  / G2 
otin revokedGrants
  / grantee[G2] = EvidenceSubject[E1]
  / grantPower[G2] in authority[issuer[G2]]
  / evidenceRecorded' = evidenceRecorded cup {E1}
  / activeGrants' = activeGrants cup {G2}
  / revokedGrants' = revokedGrants
  / authority' = AuthorityOf(activeGrants cup {G2})
  / evidenceAuthorityBefore' =
       [evidenceAuthorityBefore EXCEPT ![E1] = authority]
  / evidenceAuthorityAfter' =
       [evidenceAuthorityAfter EXCEPT ![E1] = authority']

BadSubjectMismatch ==
  / Control = "subject-mismatch"
  / E1 
otin evidenceRecorded
  / SignatureValid[E1]
  / SignerTrusted[E1]
  / EvidenceSubject[E1] # TargetSubject[E1]
  / G3 
otin activeGrants
  / G3 
otin revokedGrants
  / grantee[G3] = EvidenceSubject[E1]
  / grantPower[G3] in authority[issuer[G3]]
  / evidenceRecorded' = evidenceRecorded cup {E1}
  / activeGrants' = activeGrants cup {G3}
  / revokedGrants' = revokedGrants
  / authority' = AuthorityOf(activeGrants cup {G3})
  / evidenceAuthorityBefore' =
       [evidenceAuthorityBefore EXCEPT ![E1] = authority]
  / evidenceAuthorityAfter' =
       [evidenceAuthorityAfter EXCEPT ![E1] = authority']

NegativeNext == BadUntrustedAttestation / BadSubjectMismatch
NegativeSpec == Init / [][NegativeNext]_vars
=========================================================================
