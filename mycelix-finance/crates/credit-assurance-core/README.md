# Credit Assurance Core

A dependency-free Rust reference kernel for loan lifecycle and repayment integrity.

## Scope

- Explicit loan offer/accept/fund/activate state machine.
- Simple contractual interest accrual using integer minor units, basis points and an explicit day-count denominator; fractional interest remainder is carried so chunked accrual does not lose value.
- Payment application only against an externally final settlement receipt bound to exact loan, borrower, lender, currency, amount, day and idempotency key.
- Exact retry returns the prior allocation; changed payload and reused external receipt IDs are rejected.
- Partial repayments allocate to accrued interest first, then principal; overpayment is rejected rather than silently creating a negative balance or unallocated cash.
- Hardship, forbearance, restructuring, dispute, default and write-off are explicit recorded transitions. A write-off does not silently extinguish the borrower's legal obligation.
- Explicit affordability arithmetic and expected-credit-loss (PD × LGD × EAD) calculation. No MYCEL/reputation input or opaque universal credit score.

## Run

From `mycelix-finance/`:

`cargo test --locked -p credit-assurance-core`

## Limits

This is not a bank, lender, servicing platform, credit bureau or underwriting model. It does not authenticate bank receipts, enforce Holochain authorship, prove legal authority, establish jurisdictional compliance, estimate PD/LGD from data, or make lending decisions. Those remain the responsibility of licensed/accountable institutions and independently qualified adapters. The kernel only makes selected arithmetic and lifecycle invariants executable.
