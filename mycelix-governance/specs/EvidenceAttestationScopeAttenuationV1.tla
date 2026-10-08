---------------- MODULE EvidenceAttestationScopeAttenuationV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS Root, Alice,
          S1, S2,
          G1, G2,
          E1,
          SignatureValid, SignerTrusted, ClaimAuthorized,
          EvidenceSubject, TargetSubject,
          ClaimScope, GrantScope

Agents == {Root, Alice}
Scopes == {S1, S2}
Grants == {G1, G2}
Evidence == {E1}

issuer == [G1 |-> Root, G2 |-> Root]
grantee == [G1 |-> Alice, G2 |-> Alice]

VARIABLES activeGrants, revokedGrants, authority, evidenceRecorded,
          evidenceAuthorityBefore, evidenceAuthorityAfter

vars == <<activeGrants, revokedGrants, authority, evidenceRecorded,
          evidenceAuthorityBefore, evidenceAuthorityAfter>>

AuthorityOf(active) ==
  [a \in Agents |-> IF a = Root
                    THEN Scopes
                    ELSE UNION {GrantScope[g] : g \in active /\ grantee[g] = a}]

EmptyAuthoritySnapshot == [a \in Agents |-> {}]

Init ==
  /\ activeGrants = {G1}
  /\ revokedGrants = {}
  /\ authority = AuthorityOf({G1})
  /\ evidenceRecorded = {}
  /\ evidenceAuthorityBefore = [e \in Evidence |-> EmptyAuthoritySnapshot]
  /\ evidenceAuthorityAfter = [e \in Evidence |-> EmptyAuthoritySnapshot]

RecordEvidence(e) ==
  /\ e \in Evidence
  /\ e \notin evidenceRecorded
  /\ evidenceRecorded' = evidenceRecorded \cup {e}
  /\ evidenceAuthorityBefore' =
       [evidenceAuthorityBefore EXCEPT ![e] = authority]
  /\ evidenceAuthorityAfter' =
       [evidenceAuthorityAfter EXCEPT ![e] = authority]
  /\ UNCHANGED <<activeGrants, revokedGrants, authority>>

ApplyEvidenceBackedGrant(e, g) ==
  /\ e \in Evidence
  /\ e \notin evidenceRecorded
  /\ SignatureValid[e]
  /\ SignerTrusted[e]
  /\ ClaimAuthorized[e]
  /\ EvidenceSubject[e] = TargetSubject[e]
  /\ g \in Grants
  /\ g \notin activeGrants
  /\ g \notin revokedGrants
  /\ grantee[g] = EvidenceSubject[e]
  /\ GrantScope[g] \subseteq authority[issuer[g]]
  /\ GrantScope[g] \subseteq ClaimScope[e]
  /\ evidenceRecorded' = evidenceRecorded \cup {e}
  /\ activeGrants' = activeGrants \cup {g}
  /\ revokedGrants' = revokedGrants
  /\ authority' = AuthorityOf(activeGrants \cup {g})
  /\ evidenceAuthorityBefore' =
       [evidenceAuthorityBefore EXCEPT ![e] = authority]
  /\ evidenceAuthorityAfter' =
       [evidenceAuthorityAfter EXCEPT ![e] = authority']

Next ==
  \/ \E e \in Evidence : RecordEvidence(e)
  \/ \E e \in Evidence, g \in Grants : ApplyEvidenceBackedGrant(e, g)

TypeOK ==
  /\ activeGrants \subseteq Grants
  /\ revokedGrants \subseteq Grants
  /\ activeGrants \cap revokedGrants = {}
  /\ authority \in [Agents -> SUBSET Scopes]
  /\ evidenceRecorded \subseteq Evidence
  /\ evidenceAuthorityBefore \in [Evidence -> [Agents -> SUBSET Scopes]]
  /\ evidenceAuthorityAfter \in [Evidence -> [Agents -> SUBSET Scopes]]

AuthorityMatchesCurrentGrants == authority = AuthorityOf(activeGrants)

GrantScopeWithinIssuerAuthority ==
  \A g \in activeGrants : GrantScope[g] \subseteq authority[issuer[g]]

NoAuthorityWithoutCurrentGrant ==
  \A a \in Agents \ {Root}, s \in authority[a] :
    \E g \in activeGrants :
      grantee[g] = a /\ s \in GrantScope[g]

ClaimAuthorizationScopeAttenuated ==
  \A e \in evidenceRecorded :
    (SignatureValid[e] /\ SignerTrusted[e] /\ ClaimAuthorized[e] /\
     EvidenceSubject[e] = TargetSubject[e]) =>
      (evidenceAuthorityAfter[e][EvidenceSubject[e]]
        \ evidenceAuthorityBefore[e][EvidenceSubject[e]]) \subseteq ClaimScope[e]

=========================================================================
