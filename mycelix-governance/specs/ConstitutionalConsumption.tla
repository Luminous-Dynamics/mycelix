-------------------- MODULE ConstitutionalConsumption --------------------
EXTENDS Naturals, FiniteSets

(***************************************************************************)
(* Small safety model for MYC-CONST-003B.                                 *)
(*                                                                         *)
(* Competing claims are allowed to be pending for the same use index.      *)
(* Safety requires that at most one can finalize for that use and that no  *)
(* effect occurs without finality. Revocation prevents later finalization   *)
(* but does not erase claims that finalized before the Revoke transition.  *)
(***************************************************************************)

CONSTANTS Claims, Uses, ClaimUse, Witnessed

VARIABLES pending, finalized, effected, revoked

vars == <<pending, finalized, effected, revoked>>

TypeOK ==
    /\ pending \subseteq Claims
    /\ finalized \subseteq Claims
    /\ effected \subseteq Uses
    /\ revoked \in BOOLEAN
    /\ \A c \in Claims : ClaimUse[c] \in Uses
    /\ Witnessed \subseteq Claims

Init ==
    /\ pending = {}
    /\ finalized = {}
    /\ effected = {}
    /\ revoked = FALSE

Submit(c) ==
    /\ c \in Claims
    /\ c \notin pending
    /\ c \notin finalized
    /\ ~\E f \in finalized : ClaimUse[f] = ClaimUse[c]
    /\ pending' = pending \cup {c}
    /\ UNCHANGED <<finalized, effected, revoked>>

Finalize(c) ==
    /\ c \in pending
    /\ c \in Witnessed
    /\ ~revoked
    /\ \A f \in finalized : ClaimUse[f] # ClaimUse[c]
    /\ finalized' = finalized \cup {c}
    /\ UNCHANGED <<pending, effected, revoked>>

Revoke ==
    /\ ~revoked
    /\ revoked' = TRUE
    /\ UNCHANGED <<pending, finalized, effected>>

Execute(c) ==
    /\ c \in finalized
    /\ ClaimUse[c] \notin effected
    /\ effected' = effected \cup {ClaimUse[c]}
    /\ UNCHANGED <<pending, finalized, revoked>>

Redeliver(c) ==
    /\ c \in finalized
    /\ ClaimUse[c] \in effected
    /\ UNCHANGED vars

Next ==
    \/ \E c \in Claims : Submit(c)
    \/ \E c \in Claims : Finalize(c)
    \/ Revoke
    \/ \E c \in Claims : Execute(c)
    \/ \E c \in Claims : Redeliver(c)

Spec == Init /\ [][Next]_vars

AtMostOneFinalizedPerUse ==
    \A c1 \in finalized :
        \A c2 \in finalized :
            (ClaimUse[c1] = ClaimUse[c2]) => (c1 = c2)

NoEffectWithoutFinality ==
    \A u \in effected : \E c \in finalized : ClaimUse[c] = u

FinalizedWithinBudget == Cardinality(finalized) <= Cardinality(Uses)

=============================================================================
