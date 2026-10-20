---------------- MODULE AgentDelegationAuthorityV1NegativeControls ----------------
EXTENDS AgentDelegationAuthorityV1

CONSTANT Control

BadUndelegatedChild ==
  /\ activeGrants' = activeGrants \cup {G2}
  /\ revokedGrants' = revokedGrants
  /\ authority' = [authority EXCEPT ![B] = @ \cup {P1}]
  /\ UNCHANGED <<evidenceRecorded, evidenceAuthorityBefore,
                  evidenceAuthorityAfter, providerFailed,
                  authorityBeforeFailure>>

BadGrandchildExceedsAncestor ==
  /\ activeGrants' = {G1, G2, G3, G4}
  /\ revokedGrants' = revokedGrants
  /\ authority' = [authority EXCEPT
       ![A] = {P1},
       ![B] = {P1},
       ![C] = {P1, P2}]
  /\ UNCHANGED <<evidenceRecorded, evidenceAuthorityBefore,
                  evidenceAuthorityAfter, providerFailed,
                  authorityBeforeFailure>>

BadTransitivePower ==
  /\ activeGrants' = {G1, G2, G3, G4}
  /\ revokedGrants' = {}
  /\ authority' = [authority EXCEPT
       ![A] = {P1},
       ![B] = {P1, P2},
       ![C] = {P2}]
  /\ UNCHANGED <<evidenceRecorded, evidenceAuthorityBefore,
                  evidenceAuthorityAfter, providerFailed,
                  authorityBeforeFailure>>

BadRevokedDescendant ==
  /\ revokedGrants' = revokedGrants \cup {G1}
  /\ activeGrants' = activeGrants \cup {G1, G2}
  /\ authority' = authority
  /\ UNCHANGED <<evidenceRecorded, evidenceAuthorityBefore,
                  evidenceAuthorityAfter, providerFailed,
                  authorityBeforeFailure>>

BadEvidenceMint ==
  /\ E1 \notin evidenceRecorded
  /\ G4 \notin activeGrants
  /\ G4 \notin revokedGrants
  /\ grantPower[G4] \in authority[issuer[G4]]
  /\ evidenceRecorded' = evidenceRecorded \cup {E1}
  /\ activeGrants' = activeGrants \cup {G4}
  /\ authority' = AuthorityOf(activeGrants \cup {G4})
  /\ evidenceAuthorityBefore' =
       [evidenceAuthorityBefore EXCEPT ![E1] = authority]
  /\ evidenceAuthorityAfter' =
       [evidenceAuthorityAfter EXCEPT ![E1] = authority']
  /\ UNCHANGED <<revokedGrants, providerFailed,
                  authorityBeforeFailure>>

BadFailureAuthority ==
  /\ providerFailed' = TRUE
  /\ authorityBeforeFailure' = authority
  /\ authority' = [authority EXCEPT ![A] = @ \cup {P2}]
  /\ UNCHANGED <<activeGrants, revokedGrants, evidenceRecorded,
                  evidenceAuthorityBefore, evidenceAuthorityAfter>>

NegativeNext ==
  \/ IF Control = "transitive-power" THEN BadTransitivePower ELSE FALSE
  \/ IF Control = "undelegated-child" THEN BadUndelegatedChild ELSE FALSE
  \/ IF Control = "grandchild-exceeds-ancestor" THEN BadGrandchildExceedsAncestor ELSE FALSE
  \/ IF Control = "revoked-descendant" THEN BadRevokedDescendant ELSE FALSE
  \/ IF Control = "evidence-mint" THEN BadEvidenceMint ELSE FALSE
  \/ IF Control = "failure-authority" THEN BadFailureAuthority ELSE FALSE

NegativeSpec == Init /\ [][NegativeNext]_vars

=========================================================================