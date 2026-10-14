---------------- MODULE EvidenceAttestationScopeAttenuationV1NegativeControls ----------------
EXTENDS EvidenceAttestationScopeAttenuationV1

CONSTANT Control

BadScopeExpansion ==
  /\ Control = "scope-expansion"
  /\ E1 \notin evidenceRecorded
  /\ SignatureValid[E1]
  /\ SignerTrusted[E1]
  /\ ClaimAuthorized[E1]
  /\ EvidenceSubject[E1] = TargetSubject[E1]
  /\ ClaimScope[E1] = {S1}
  /\ G2 \notin activeGrants
  /\ G2 \notin revokedGrants
  /\ grantee[G2] = EvidenceSubject[E1]
  /\ GrantScope[G2] = {S1, S2}
  /\ GrantScope[G2] \subseteq authority[issuer[G2]]
  /\ evidenceRecorded' = evidenceRecorded \cup {E1}
  /\ activeGrants' = activeGrants \cup {G2}
  /\ revokedGrants' = revokedGrants
  /\ authority' = AuthorityOf(activeGrants \cup {G2})
  /\ evidenceAuthorityBefore' =
       [evidenceAuthorityBefore EXCEPT ![E1] = authority]
  /\ evidenceAuthorityAfter' =
       [evidenceAuthorityAfter EXCEPT ![E1] = authority']

NegativeNext == BadScopeExpansion
NegativeSpec == Init /\ [][NegativeNext]_vars
=========================================================================
