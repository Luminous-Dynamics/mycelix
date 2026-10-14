---------------- MODULE EvidenceAttestationProvenanceV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS Root, Alice, Bob,
          P1, P2,
          G1, G2, G3,
          E1,
          SignatureValid,
          SignerTrusted,
          EvidenceSubject,
          TargetSubject,
          GrantPower

Agents == {Root, Alice, Bob}
Powers == {P1, P2}
Grants == {G1, G2, G3}
Evidence == {E1}

issuer == [G1 |-> Root, G2 |-> Root, G3 |-> Root]
grantee == [G1 |-> Alice, G2 |-> Alice, G3 |-> Bob]
grantPower == GrantPower
RootPowers == {P1, P2}

VARIABLES activeGrants, revokedGrants, authority, evidenceRecorded,
          evidenceAuthorityBefore, evidenceAuthorityAfter

vars == <<activeGrants, revokedGrants, authority, evidenceRecorded,
          evidenceAuthorityBefore, evidenceAuthorityAfter>>

AuthorityOf(active) ==
  [a in Agents |-> IF a = Root
                    THEN RootPowers
                    ELSE {grantPower[g] : g in active / grantee[g] = a}]

EmptyAuthoritySnapshot == [a in Agents |-> {}]

Init ==
  / activeGrants = {G1}
  / revokedGrants = {}
  / authority = AuthorityOf({G1})
  / evidenceRecorded = {}
  / evidenceAuthorityBefore = [e in Evidence |-> EmptyAuthoritySnapshot]
  / evidenceAuthorityAfter = [e in Evidence |-> EmptyAuthoritySnapshot]

RecordEvidence(e) ==
  / e in Evidence
  / e 
otin evidenceRecorded
  / evidenceRecorded' = evidenceRecorded cup {e}
  / evidenceAuthorityBefore' =
       [evidenceAuthorityBefore EXCEPT ![e] = authority]
  / evidenceAuthorityAfter' =
       [evidenceAuthorityAfter EXCEPT ![e] = authority]
  / UNCHANGED <<activeGrants, revokedGrants, authority>>

ApplyEvidenceBackedGrant(e, g) ==
  / e in Evidence
  / e 
otin evidenceRecorded
  / SignatureValid[e]
  / SignerTrusted[e]
  / EvidenceSubject[e] = TargetSubject[e]
  / g in Grants
  / g 
otin activeGrants
  / g 
otin revokedGrants
  / grantee[g] = EvidenceSubject[e]
  / grantPower[g] in authority[issuer[g]]
  / evidenceRecorded' = evidenceRecorded cup {e}
  / activeGrants' = activeGrants cup {g}
  / revokedGrants' = revokedGrants
  / authority' = AuthorityOf(activeGrants cup {g})
  / evidenceAuthorityBefore' =
       [evidenceAuthorityBefore EXCEPT ![e] = authority]
  / evidenceAuthorityAfter' =
       [evidenceAuthorityAfter EXCEPT ![e] = authority']

Next ==
  / E e in Evidence : RecordEvidence(e)
  / E e in Evidence, g in Grants : ApplyEvidenceBackedGrant(e, g)

TypeOK ==
  / activeGrants subseteq Grants
  / revokedGrants subseteq Grants
  / activeGrants cap revokedGrants = {}
  / authority in [Agents -> SUBSET Powers]
  / evidenceRecorded subseteq Evidence
  / evidenceAuthorityBefore in [Evidence -> [Agents -> SUBSET Powers]]
  / evidenceAuthorityAfter in [Evidence -> [Agents -> SUBSET Powers]]

AuthorityMatchesCurrentGrants == authority = AuthorityOf(activeGrants)

GrantCannotExceedIssuerAuthority ==
  A g in activeGrants : grantPower[g] in authority[issuer[g]]

NoAuthorityWithoutCurrentGrant ==
  A a in Agents  {Root}, p in authority[a] :
    E g in activeGrants : grantee[g] = a / grantPower[g] = p

UntrustedAttestationCannotChangeAuthority ==
  A e in evidenceRecorded :
    (SignatureValid[e] / ~SignerTrusted[e]) =>
      evidenceAuthorityAfter[e] = evidenceAuthorityBefore[e]

SubjectMismatchCannotChangeAuthority ==
  A e in evidenceRecorded :
    (SignatureValid[e] / SignerTrusted[e] / EvidenceSubject[e] # TargetSubject[e]) =>
      evidenceAuthorityAfter[e] = evidenceAuthorityBefore[e]

=========================================================================