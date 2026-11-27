---------------- MODULE TemporalProgressAntichain ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS
    OwnerA,
    OwnerB,
    NoOwner

ASSUME OwnerA # OwnerB
ASSUME OwnerA # NoOwner
ASSUME OwnerB # NoOwner

Cut(a, b) == <<a, b>>
InitialFrontier == {Cut(1, 0), Cut(0, 1)}
AdvancedFrontier == {Cut(2, 0), Cut(0, 2)}
CompletedFrontier == {}
AllowedFrontiers == {InitialFrontier, AdvancedFrontier, CompletedFrontier}

LE(left, right) ==
    /\ left[1] <= right[1]
    /\ left[2] <= right[2]

IsAntichain(frontier) ==
    \A left, right \in frontier:
        left = right \/ (~LE(left, right) /\ ~LE(right, left))

VARIABLES frontier, previousFrontier, queryCut, stable

vars == <<frontier, previousFrontier, queryCut, stable>>

Init ==
    /\ frontier = InitialFrontier
    /\ previousFrontier = InitialFrontier
    /\ queryCut = Cut(1, 1)
    /\ stable = FALSE

AdvanceFrontier ==
    \E next \in AllowedFrontiers:
        /\ next # frontier
        /\ IsAntichain(next)
        /\ \A point \in next:
              \E prior \in frontier: LE(prior, point)
        /\ previousFrontier' = frontier
        /\ frontier' = next
        /\ UNCHANGED <<queryCut, stable>>

MarkStable ==
    /\ ~stable
    /\ ~\E point \in frontier: LE(point, queryCut)
    /\ stable' = TRUE
    /\ UNCHANGED <<frontier, previousFrontier, queryCut>>

KeepStable ==
    /\ stable
    /\ UNCHANGED vars

Next == AdvanceFrontier \/ MarkStable \/ KeepStable

(* Deliberately faulty finalizer: it ignores outstanding progress points. *)
MutantMarkStable ==
    /\ ~stable
    /\ stable' = TRUE
    /\ UNCHANGED <<frontier, previousFrontier, queryCut>>

MutantNext == AdvanceFrontier \/ MarkStable \/ MutantMarkStable \/ KeepStable

Spec == Init /\ [][Next]_vars
MutantSpec == Init /\ [][MutantNext]_vars

TypeOK ==
    /\ frontier \subseteq AllowedPoints
    /\ previousFrontier \subseteq AllowedPoints
    /\ queryCut \in AllowedPoints
    /\ stable \in BOOLEAN

AllowedPoints == {Cut(0, 0), Cut(0, 1), Cut(0, 2),
                  Cut(1, 0), Cut(1, 1), Cut(1, 2),
                  Cut(2, 0), Cut(2, 1), Cut(2, 2)}

FrontierIsAntichain == IsAntichain(frontier)
PreviousFrontierIsAntichain == IsAntichain(previousFrontier)

FrontierAdvanceMonotone ==
    \A point \in frontier:
        \E prior \in previousFrontier: LE(prior, point)

StableSound ==
    stable => ~\E point \in frontier: LE(point, queryCut)

=============================================================================
