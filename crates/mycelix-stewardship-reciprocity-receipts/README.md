# Mycelix Stewardship Reciprocity Receipts — STEW-011

This crate records evidence-bearing **reports about reciprocal performance** without converting a report into final satisfaction.

```text
receipt exists
!= performance verified
!= beneficiary accepted performance
!= obligation satisfied
!= dispute resolved
```

Multiple receipts for the same obligation may coexist, including conflicting reports. There is deliberately no `latest wins`, score, or automatic satisfaction reducer.

A receipt references the STEW-010 obligation, the reporting principal, an asserted performer, an event reference, an explicit reported outcome, and one or more evidence references.

`ReportedFulfilled` means exactly that: someone reported fulfillment. It is not named `Fulfilled` because the receipt theorem cannot establish that stronger fact.
