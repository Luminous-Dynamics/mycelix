---------------- MODULE AgentDelegationAuthorityV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS Root, A, B, C,
          P1, P2,
          G1, G2, G3, G4,
          E1,
          ProviderX,
          GrantPower

Agents == {Root, A, B, C}
Powers == {P1, P2}
Grants == {G1, G2, G3, G4}
Evidence == {E1}

issuer ==
  [G1 |-> Root,
   G2 |-> A,
   G3 |-> B,
   G4 |-> Root]

grantee ==
  [G1 |-> A,
   G2 |-> B,
   G3 |-> C,
   G4 |-> B]

grantPower == GrantPower

ancestor ==
  [G1 |-> {},
   G2 |-> {G1},
   G3 |-> {G1, G2},
   G4 |-> {}]

RootPowers == {P1, P2}

VARIABLES activeGrants,
          revokedGrants,
          authority,
          evidenceRecorded,
          evidenceAuthorityBefore,
          evidenceAuthorityAfter,
          providerFailed,
          authorityBeforeFailure

vars ==
  <<activeGrants, revokedGrants, authority, evidenceRecorded,
    evidenceAuthorityBefore, evidenceAuthorityAfter,
    providerFailed, authorityBeforeFailure>>

AuthorityOf(active) ==
  [a \in Agents |-> IF a = Root
                    THEN RootPowers
                    ELSE {grantPower[g] : g \in active /\ grantee[g] = a}]

EmptyAuthoritySnapshot == [a \in Agents |-> {}]

Init ==
  /\ activeGrants = {}
  /\ revokedGrants = {}
  /\ authority = AuthorityOf({})
  /\ evidenceRecorded = {}
  /\ evidenceAuthorityBefore = [e \in Evidence |-> EmptyAuthoritySnapshot]
  /\ evidenceAuthorityAfter = [e \in Evidence |-> EmptyAuthoritySnapshot]
  /\ providerFailed = FALSE
  /\ authorityBeforeFailure = authority

ActivateGrant(g) ==
  /\ g \in Grants
  /\ g \notin activeGrants
  /\ g \notin revokedGrants
  /\ grantPower[g] \in authority[issuer[g]]
  /\ activeGrants' = activeGrants \cup {g}
  /\ revokedGrants' = revokedGrants
  /\ authority' = AuthorityOf(activeGrants \cup {g})
  /\ UNCHANGED <<evidenceRecorded, evidenceAuthorityBefore,
                  evidenceAuthorityAfter, providerFailed,
                  authorityBeforeFailure>>

RevokeGrant(g) ==
  /\ g \in Grants
  /\ g \in activeGrants
  /\ revokedGrants' = revokedGrants \cup {g}
  /\ activeGrants' =
       {h \in activeGrants : h # g /\ g \notin ancestor[h]}
  /\ authority' = AuthorityOf(activeGrants')
  /\ UNCHANGED <<evidenceRecorded, evidenceAuthorityBefore,
                  evidenceAuthorityAfter, providerFailed,
                  authorityBeforeFailure>>

RecordEvidence(e) ==
  /\ e \in Evidence
  /\ e \notin evidenceRecorded
  /\ evidenceRecorded' = evidenceRecorded \cup {e}
  /\ evidenceAuthorityBefore' =
       [evidenceAuthorityBefore EXCEPT ![e] = authority]
  /\ evidenceAuthorityAfter' =
       [evidenceAuthorityAfter EXCEPT ![e] = authority]
  /\ UNCHANGED <<activeGrants, revokedGrants, authority,
                  providerFailed, authorityBeforeFailure>>

ProviderFailure ==
  /\ ~providerFailed
  /\ providerFailed' = TRUE
  /\ authorityBeforeFailure' = authority
  /\ UNCHANGED <<activeGrants, revokedGrants, authority,
                  evidenceRecorded, evidenceAuthorityBefore,
                  evidenceAuthorityAfter>>

Next ==
  \/ \E g \in Grants : ActivateGrant(g)
  \/ \E g \in Grants : RevokeGrant(g)
  \/ \E e \in Evidence : RecordEvidence(e)
  \/ ProviderFailure

TypeOK ==
  /\ activeGrants \subseteq Grants
  /\ revokedGrants \subseteq Grants
  /\ activeGrants \cap revokedGrants = {}
  /\ authority \in [Agents -> SUBSET Powers]
  /\ evidenceRecorded \subseteq Evidence
  /\ evidenceAuthorityBefore \in [Evidence -> [Agents -> SUBSET Powers]]
  /\ evidenceAuthorityAfter \in [Evidence -> [Agents -> SUBSET Powers]]
  /\ providerFailed \in BOOLEAN
  /\ authorityBeforeFailure \in [Agents -> SUBSET Powers]

AuthorityMatchesCurrentGrants ==
  authority = AuthorityOf(activeGrants)

DelegationNonAmplification ==
  \A g \in activeGrants :
    grantPower[g] \in authority[issuer[g]]

TransitiveDelegationBounded ==
  \A g \in activeGrants :
    \A a \in ancestor[g] :
      grantPower[g] \in authority[issuer[a]]

RevocationPropagates ==
  \A g \in revokedGrants :
    \A h \in activeGrants : g \notin ancestor[h]

EvidenceDoesNotMintAuthority ==
  \A e \in evidenceRecorded :
    evidenceAuthorityAfter[e] = evidenceAuthorityBefore[e]

FailureDoesNotMintAuthority ==
  providerFailed => authority = authorityBeforeFailure

NoAuthorityWithoutCurrentGrant ==
  \A a \in Agents \ {Root}, p \in authority[a] :
    \E g \in activeGrants : grantee[g] = a /\ grantPower[g] = p

=========================================================================