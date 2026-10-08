---------------- MODULE EvidenceAttestationClaimAuthorizationV1NegativeControls ----------------
EXTENDS EvidenceAttestationClaimAuthorizationV1

CONSTANT Control

BadUnauthorizedClaim ==
  / Control = "unauthorized-claim"
  / E1 
otin evidenceRecorded
  / SignatureValid[E1]
  / SignerTrusted[E1]
  / ~ClaimAuthorized[E1]
  / EvidenceSubject[E1] = TargetSubject[E1]
  / ClaimType[E1] = ClaimA
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

NegativeNext == BadUnauthorizedClaim
NegativeSpec == Init / [][NegativeNext]_vars
=========================================================================
