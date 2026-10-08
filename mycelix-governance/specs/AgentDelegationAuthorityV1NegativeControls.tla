---------------- MODULE AgentDelegationAuthorityV1NegativeControls ----------------
EXTENDS AgentDelegationAuthorityV1

CONSTANT Control

BadUndelegatedChild ==
  /\ activeGrants' = activeGrants \cup {G2}
  /\ revokedGrants' = revokedGrants
  /\ authority' = [authority EXCEPT ![B] = @ \cup {P2}]
  /\ UNCHANGED <<evidenceRecorded, providerFailed, authorityBeforeFailure>>

BadGrandchildExceedsAncestor ==
  /\ activeGrants' = activeGrants \cup {G3}
  /\ revokedGrants' = revokedGrants
  /\ authority' = [authority EXCEPT ![C] = @ \cup {P2}]
  /\ UNCHANGED <<evidenceRecorded, providerFailed, authorityBeforeFailure>>

BadRevokedDescendant ==
  /\ revokedGrants' = revokedGrants \cup {G1}
  /\ activeGrants' = activeGrants \cup {G1, G2}
  /\ authority' = authority
  /\ UNCHANGED <<evidenceRecorded, providerFailed, authorityBeforeFailure>>

BadEvidenceMint ==
  /\ evidenceRecorded' = evidenceRecorded \cup {E1}
  /\ authority' = [authority EXCEPT ![B] = @ \cup {P2}]
  /\ UNCHANGED <<activeGrants, revokedGrants, providerFailed, authorityBeforeFailure>>

BadFailureAuthority ==
  /\ providerFailed' = TRUE
  /\ authorityBeforeFailure' = authority
  /\ authority' = [authority EXCEPT ![A] = @ \cup {P2}]
  /\ UNCHANGED <<activeGrants, revokedGrants, evidenceRecorded>>

NegativeNext ==
  \/ IF Control = "undelegated-child" THEN BadUndelegatedChild ELSE FALSE
  \/ IF Control = "grandchild-exceeds-ancestor" THEN BadGrandchildExceedsAncestor ELSE FALSE
  \/ IF Control = "revoked-descendant" THEN BadRevokedDescendant ELSE FALSE
  \/ IF Control = "evidence-mint" THEN BadEvidenceMint ELSE FALSE
  \/ IF Control = "failure-authority" THEN BadFailureAuthority ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars

=========================================================================
