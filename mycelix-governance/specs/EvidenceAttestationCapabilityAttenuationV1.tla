---------------- MODULE EvidenceAttestationCapabilityAttenuationV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS Root, Alice,
          R1, R2,
          Read, Write,
          AudienceA, AudienceB,
          T1, T2,
          G1, G2,
          E1,
          SignatureValid, SignerTrusted, ClaimAuthorized,
          EvidenceSubject, TargetSubject,
          ClaimResources, ClaimActions, ClaimAudiences, ClaimExpiry,
          GrantResource, GrantAction, GrantAudience, GrantExpiry

Agents == {Root, Alice}
Resources == {R1, R2}
Actions == {Read, Write}
Audiences == {AudienceA, AudienceB}
Times == {T1, T2}
Grants == {G1, G2}
Evidence == {E1}

issuer == [G1 |-> Root, G2 |-> Root]
grantee == [G1 |-> Alice, G2 |-> Alice]
TimeRank == [T1 |-> 1, T2 |-> 2]

Capability(g) ==
  [resource |-> GrantResource[g],
   action |-> GrantAction[g],
   audience |-> GrantAudience[g],
   expiry |-> GrantExpiry[g]]

CapabilityUniverse ==
  { [resource |-> r, action |-> a, audience |-> u, expiry |-> t] :
      r \in Resources, a \in Actions, u \in Audiences, t \in Times }

VARIABLES activeGrants, revokedGrants, authority, evidenceRecorded,
          evidenceAuthorityBefore, evidenceAuthorityAfter

vars == <<activeGrants, revokedGrants, authority, evidenceRecorded,
          evidenceAuthorityBefore, evidenceAuthorityAfter>>

AuthorityOf(active) ==
  [a \in Agents |-> IF a = Root
                    THEN CapabilityUniverse
                    ELSE {Capability(g) :
                            g \in active /\ grantee[g] = a}]

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
  /\ Capability(g) \in authority[issuer[g]]
  /\ GrantResource[g] \in ClaimResources[e]
  /\ GrantAction[g] \in ClaimActions[e]
  /\ GrantAudience[g] \in ClaimAudiences[e]
  /\ TimeRank[GrantExpiry[g]] <= TimeRank[ClaimExpiry[e]]
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
  /\ authority \in [Agents -> SUBSET CapabilityUniverse]
  /\ evidenceRecorded \subseteq Evidence
  /\ evidenceAuthorityBefore \in [Evidence -> [Agents -> SUBSET CapabilityUniverse]]
  /\ evidenceAuthorityAfter \in [Evidence -> [Agents -> SUBSET CapabilityUniverse]]

AuthorityMatchesCurrentGrants ==
  authority = AuthorityOf(activeGrants)

GrantCapabilitiesWithinIssuerAuthority ==
  \A g \in activeGrants : Capability(g) \in authority[issuer[g]]

NoAuthorityWithoutCurrentGrant ==
  \A a \in Agents \ {Root}, c \in authority[a] :
    \E g \in activeGrants :
      grantee[g] = a /\ Capability(g) = c

ResourceScopeAttenuated ==
  \A e \in evidenceRecorded :
    (SignatureValid[e] /\ SignerTrusted[e] /\ ClaimAuthorized[e] /\
     EvidenceSubject[e] = TargetSubject[e]) =>
      \A c \in evidenceAuthorityAfter[e][EvidenceSubject[e]]
                \ evidenceAuthorityBefore[e][EvidenceSubject[e]] :
        c.resource \in ClaimResources[e]

ActionScopeAttenuated ==
  \A e \in evidenceRecorded :
    (SignatureValid[e] /\ SignerTrusted[e] /\ ClaimAuthorized[e] /\
     EvidenceSubject[e] = TargetSubject[e]) =>
      \A c \in evidenceAuthorityAfter[e][EvidenceSubject[e]]
                \ evidenceAuthorityBefore[e][EvidenceSubject[e]] :
        c.action \in ClaimActions[e]

AudienceScopeAttenuated ==
  \A e \in evidenceRecorded :
    (SignatureValid[e] /\ SignerTrusted[e] /\ ClaimAuthorized[e] /\
     EvidenceSubject[e] = TargetSubject[e]) =>
      \A c \in evidenceAuthorityAfter[e][EvidenceSubject[e]]
                \ evidenceAuthorityBefore[e][EvidenceSubject[e]] :
        c.audience \in ClaimAudiences[e]

ExpiryScopeAttenuated ==
  \A e \in evidenceRecorded :
    (SignatureValid[e] /\ SignerTrusted[e] /\ ClaimAuthorized[e] /\
     EvidenceSubject[e] = TargetSubject[e]) =>
      \A c \in evidenceAuthorityAfter[e][EvidenceSubject[e]]
                \ evidenceAuthorityBefore[e][EvidenceSubject[e]] :
        TimeRank[c.expiry] <= TimeRank[ClaimExpiry[e]]

=========================================================================
