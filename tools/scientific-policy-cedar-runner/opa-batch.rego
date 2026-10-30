package mycelix.authz.batch

import rego.v1

# Evaluate every frozen abstract context through the unmodified pure policy
# module. The wrapper changes only input for each case and never synthesizes
# an expected decision.
decisions := [
    {
        "case_id": item.case_id,
        "decision": decision,
    } |
    some item in input.cases
    decision := data.mycelix.authz.decision with input as item.input
]
