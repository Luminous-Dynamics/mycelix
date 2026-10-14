---------------- MODULE EvidenceAttestationCapabilityAttenuationV1NegativeControls ----------------
EXTENDS EvidenceAttestationCapabilityAttenuationV1

CONSTANT Control

BaseBadTransition(e, g) ==
  /\ e = E1
  /\ g = G2
  /\ e \notin evidenceRecorded
  /\ g \notin activeGrants
  /\ g \notin revokedGrants
  /\ SignatureValid[e]
  /\ SignerTrusted[e]
  /\ ClaimAuthorized[e]
  /\ EvidenceSubject[e] = TargetSubject[e]
  /\ grantee[g] = EvidenceSubject[e]
  /\ Capability(g) \in authority[issuer[g]]
  /\ evidenceRecorded' = evidenceRecorded \cup {e}
  /\ activeGrants' = activeGrants \cup {g}
  /\ revokedGrants' = revokedGrants
  /\ authority' = AuthorityOf(activeGrants \cup {g})
  /\ evidenceAuthorityBefore' =
       [evidenceAuthorityBefore EXCEPT ![e] = authority]
  /\ evidenceAuthorityAfter' =
       [evidenceAuthorityAfter EXCEPT ![e] = authority']

BadResourceExpansion ==
  /\ Control = "resource-expansion"
  /\ BaseBadTransition(E1, G2)

BadActionExpansion ==
  /\ Control = "action-expansion"
  /\ BaseBadTransition(E1, G2)

BadAudienceExpansion ==
  /\ Control = "audience-expansion"
  /\ BaseBadTransition(E1, G2)

BadExpiryExpansion ==
  /\ Control = "expiry-expansion"
  /\ BaseBadTransition(E1, G2)

NegativeNext ==
  \/ BadResourceExpansion
  \/ BadActionExpansion
  \/ BadAudienceExpansion
  \/ BadExpiryExpansion

NegativeSpec == Init /\ [][NegativeNext]_vars
=========================================================================
