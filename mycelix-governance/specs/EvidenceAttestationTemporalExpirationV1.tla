---------------- MODULE EvidenceAttestationTemporalExpirationV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS Root, Alice,
          R1,
          Read,
          AudienceA,
          T1, T2, T3,
          G1,
          E1,
          SignatureValid, SignerTrusted, ClaimAuthorized,
          EvidenceSubject, TargetSubject,
          ClaimExpiry,
          GrantResource, GrantAction, GrantAudience, GrantExpiry

Agents == {Root, Alice}
Resources == {R1}
Actions == {Read}
Audiences == {AudienceA}
Times == {T1, T2, T3}
Grants == {G1}
Evidence == {E1}

issuer == [G1 |-> Root]
grantee == [G1 |-> Alice]
TimeRank == [T1 |-> 1, T2 |-> 2, T3 |-> 3]
NextTime == [T1 |-> T2, T2 |-> T3]

Capability(g) ==
  [resource |-> GrantResource[g],
   action |-> GrantAction[g],
   audience |-> GrantAudience[g],
   expiry |-> GrantExpiry[g]]

CapabilityUniverse ==
  { [resource |-> r, action |-> a, audience |-> u, expiry |-> t] :
      r \in Resources, a \in Actions, u \in Audiences, t \in Times }

VARIABLES activeGrants, revokedGrants, authority, effectiveAuthority,
          now, evidenceRecorded

vars == <<activeGrants, revokedGrants, authority, effectiveAuthority,
          now, evidenceRecorded>>

StructuralAuthorityOf(active) ==
  [a \in Agents |-> IF a = Root
                    THEN CapabilityUniverse
                    ELSE {Capability(g) :
                            g \in active /\ grantee[g] = a}]

EffectiveAuthorityOf(active, currentTime) ==
  [a \in Agents |-> IF a = Root
                    THEN CapabilityUniverse
                    ELSE {Capability(g) :
                            g \in active /\ grantee[g] = a /\
                            TimeRank[GrantExpiry[g]] >= TimeRank[currentTime]}]

Init ==
  /\ activeGrants = {G1}
  /\ revokedGrants = {}
  /\ authority = StructuralAuthorityOf({G1})
  /\ effectiveAuthority = EffectiveAuthorityOf({G1}, T1)
  /\ now = T1
  /\ evidenceRecorded = {E1}

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
  /\ TimeRank[GrantExpiry[g]] <= TimeRank[ClaimExpiry[e]]
  /\ TimeRank[GrantExpiry[g]] >= TimeRank[now]
  /\ Capability(g) \in authority[issuer[g]]
  /\ evidenceRecorded' = evidenceRecorded \cup {e}
  /\ activeGrants' = activeGrants \cup {g}
  /\ revokedGrants' = revokedGrants
  /\ authority' = StructuralAuthorityOf(activeGrants \cup {g})
  /\ effectiveAuthority' =
       EffectiveAuthorityOf(activeGrants \cup {g}, now)
  /\ now' = now

Tick ==
  /\ now # T3
  /\ now' = NextTime[now]
  /\ activeGrants' = activeGrants
  /\ revokedGrants' = revokedGrants
  /\ authority' = authority
  /\ effectiveAuthority' = EffectiveAuthorityOf(activeGrants, NextTime[now])
  /\ UNCHANGED evidenceRecorded

Next ==
  \/ \E e \in Evidence, g \in Grants : ApplyEvidenceBackedGrant(e, g)
  \/ Tick

TypeOK ==
  /\ activeGrants \subseteq Grants
  /\ revokedGrants \subseteq Grants
  /\ activeGrants \cap revokedGrants = {}
  /\ authority \in [Agents -> SUBSET CapabilityUniverse]
  /\ effectiveAuthority \in [Agents -> SUBSET CapabilityUniverse]
  /\ now \in Times
  /\ evidenceRecorded \subseteq Evidence

StructuralAuthorityMatchesCurrentGrants ==
  authority = StructuralAuthorityOf(activeGrants)

EffectiveAuthorityMatchesCurrentTime ==
  effectiveAuthority = EffectiveAuthorityOf(activeGrants, now)

GrantCapabilitiesRemainStructurallyValid ==
  \A g \in activeGrants : Capability(g) \in authority[issuer[g]]

RecordedEvidenceValid ==
  \A e \in evidenceRecorded :
    SignatureValid[e] /\ SignerTrusted[e] /\ ClaimAuthorized[e] /\
    EvidenceSubject[e] = TargetSubject[e]

NoAuthorityWithoutCurrentGrant ==
  \A a \in Agents \ {Root}, c \in authority[a] :
    \E g \in activeGrants :
      grantee[g] = a /\ Capability(g) = c

ExpiredGrantNotEffective ==
  \A g \in activeGrants :
    TimeRank[GrantExpiry[g]] < TimeRank[now] =>
      Capability(g) \notin effectiveAuthority[grantee[g]]

=========================================================================
