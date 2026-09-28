; IF01-FV-010..012 bounded semantic model.
; Status: executable formal-model witness; not yet a qualification receipt.
; Intended solver: Z3 SMT-LIB2.
;
; The model encodes only the candidate semantic predicates. It does not model
; cryptographic authentication, network transport, or production execution.

(set-option :produce-proofs true)
(set-logic ALL)

; IF01-FV-010: certification failure is distinct from authorization failure.
(declare-const certified Bool)
(declare-const authorized Bool)
(define-fun semantically_admissible () Bool
  (and certified authorized))
(push)
(assert (not certified))
(assert (semantically_admissible))
(check-sat)
(pop)

; IF01-FV-011: granted authorization requires one matching explicit reference.
(declare-const presented_ref String)
(declare-const envelope_ref String)
(declare-const required_ref String)
(define-fun authorization_binding_valid () Bool
  (and (not (= presented_ref ""))
       (= presented_ref envelope_ref)
       (= presented_ref required_ref)))
(push)
(assert (authorization_binding_valid))
(assert (not (= presented_ref envelope_ref)))
(check-sat)
(pop)

; IF01-FV-012: known recipient rejection is not indeterminate.
(declare-datatypes () ((DeliveryState TransportAccepted Delivered SemanticAdmitted Rejected Indeterminate)))
(declare-const state DeliveryState)
(define-fun semantic_admission_possible () Bool
  (= state SemanticAdmitted))
(push)
(assert (= state Rejected))
(assert semantic_admission_possible)
(check-sat)
(pop)

; Expected result for each check-sat: unsat.
