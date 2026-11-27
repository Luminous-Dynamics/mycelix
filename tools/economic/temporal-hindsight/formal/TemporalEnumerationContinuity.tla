---------------- MODULE TemporalEnumerationContinuity ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS Page0, Page1, Page2, Cursor0, Cursor1, NoCursor
Pages == {Page0, Page1, Page2}
Cursors == {Cursor0, Cursor1}

ASSUME Page0 # Page1
ASSUME Page0 # Page2
ASSUME Page1 # Page2
ASSUME Cursor0 # Cursor1
ASSUME Cursor0 # NoCursor
ASSUME Cursor1 # NoCursor

VARIABLES observedPages, expectedCursor, seenCursors,
          terminalPages, chainWellFormed

vars == <<observedPages, expectedCursor, seenCursors,
          terminalPages, chainWellFormed>>

Init ==
    /\ observedPages = {}
    /\ expectedCursor = NoCursor
    /\ seenCursors = {}
    /\ terminalPages = {}
    /\ chainWellFormed = FALSE

ObservePage0 ==
    /\ observedPages = {}
    /\ observedPages' = {Page0}
    /\ expectedCursor' = Cursor0
    /\ seenCursors' = {Cursor0}
    /\ UNCHANGED <<terminalPages, chainWellFormed>>

ObservePage1 ==
    /\ Page0 \in observedPages
    /\ Page1 \notin observedPages
    /\ expectedCursor = Cursor0
    /\ observedPages' = observedPages \cup {Page1}
    /\ expectedCursor' = Cursor1
    /\ seenCursors' = seenCursors \cup {Cursor1}
    /\ UNCHANGED <<terminalPages, chainWellFormed>>

ObservePage2 ==
    /\ Page1 \in observedPages
    /\ Page2 \notin observedPages
    /\ expectedCursor = Cursor1
    /\ observedPages' = observedPages \cup {Page2}
    /\ expectedCursor' = NoCursor
    /\ terminalPages' = terminalPages \cup {Page2}
    /\ UNCHANGED <<seenCursors, chainWellFormed>>

FinalizeEnumeration ==
    /\ observedPages = Pages
    /\ expectedCursor = NoCursor
    /\ terminalPages = {Page2}
    /\ chainWellFormed = FALSE
    /\ chainWellFormed' = TRUE
    /\ UNCHANGED <<observedPages, expectedCursor, seenCursors, terminalPages>>

Next ==
    \/ ObservePage0
    \/ ObservePage1
    \/ ObservePage2
    \/ FinalizeEnumeration

(* Faulty implementation: marks the enumeration complete after observing only
   the first page, despite an unresolved continuation cursor. *)
FinalizeTruncatedMutant ==
    /\ observedPages = {Page0}
    /\ chainWellFormed = FALSE
    /\ chainWellFormed' = TRUE
    /\ UNCHANGED <<observedPages, expectedCursor, seenCursors, terminalPages>>

MutantNext == Next \/ FinalizeTruncatedMutant

Spec == Init /\ [][Next]_vars
MutantSpec == Init /\ [][MutantNext]_vars

TypeOK ==
    /\ observedPages \subseteq Pages
    /\ expectedCursor \in Cursors \cup {NoCursor}
    /\ seenCursors \subseteq Cursors
    /\ terminalPages \subseteq Pages
    /\ chainWellFormed \in BOOLEAN

CompletePageSet ==
    chainWellFormed => observedPages = Pages

CursorChainClosed ==
    chainWellFormed =>
      /\ expectedCursor = NoCursor
      /\ seenCursors = Cursors

ExactlyOneTerminalPage ==
    chainWellFormed => terminalPages = {Page2}

=============================================================================
