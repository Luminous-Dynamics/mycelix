package sagaverifier

import (
	"fmt"
	"sync/atomic"
	"testing"
)

func benchmarkCompensationStateV2(b *testing.B) (*CompensationBookV2, CompensationAuthorizationV2) {
	b.Helper()
	forward := ForwardEffectV2{
		OperationID: "operation:benchmark",
		EffectID: "effect:benchmark",
		SourceOwnerID: "owner:merchant",
		SourceAccountID: "account:merchant",
		DestinationOwnerID: "owner:customer",
		DestinationAccountID: "account:customer",
		Asset: "USD",
		AmountMinor: ^uint64(0),
		Finality: ForwardSettledV2,
		ReversibleCeilingMinor: ^uint64(0),
	}
	forwardDigest, err := forward.CanonicalDigest()
	if err != nil { b.Fatal(err) }
	plan := CompensationPlanV2{
		PlanID: "plan:benchmark",
		ForwardEffectID: forward.EffectID,
		ForwardEffectDigest: forwardDigest,
		FundingOwnerID: "owner:treasury",
		FundingAccountID: "account:treasury",
		BeneficiaryOwnerID: forward.SourceOwnerID,
		BeneficiaryAccountID: forward.SourceAccountID,
		Asset: forward.Asset,
		MaximumCompensationMinor: ^uint64(0),
		PolicyDigest: compensationTestDigestV2('b'),
		RecoveryReason: "benchmark",
	}
	planDigest, err := plan.CanonicalDigest(forward)
	if err != nil { b.Fatal(err) }
	authorization := CompensationAuthorizationV2{
		CapabilityID: "capability:benchmark",
		IssuerOwnerID: plan.FundingOwnerID,
		GranteeOwnerID: "owner:refund-operator",
		PlanDigest: planDigest,
		FundingOwnerID: plan.FundingOwnerID,
		FundingAccountID: plan.FundingAccountID,
		BeneficiaryOwnerID: plan.BeneficiaryOwnerID,
		BeneficiaryAccountID: plan.BeneficiaryAccountID,
		Asset: plan.Asset,
		MaximumMinor: ^uint64(0),
		Nonce: 1,
		ExpiresAtUnixMS: ^uint64(0),
	}
	book, err := NewCompensationBookV2(forward, plan, ^uint64(0))
	if err != nil { b.Fatal(err) }
	return book, authorization
}

func benchmarkCompensationRequestV2(
	index uint64,
	authorization CompensationAuthorizationV2,
) CompensationRequestV2 {
	id := fmt.Sprintf("benchmark:%d", index)
	return CompensationRequestV2{
		CompensationID: id,
		ActorOwnerID: authorization.GranteeOwnerID,
		AmountMinor: 1,
		Authorization: authorization,
		LiquidityReservation: LiquidityReservationV2{
			ReservationID: "reservation:" + id,
			SourceOwnerID: authorization.FundingOwnerID,
			SourceAccountID: authorization.FundingAccountID,
			Asset: authorization.Asset,
			AmountMinor: 1,
		},
	}
}

// These benchmarks include canonical hashing, indexed identity/ceiling checks,
// and in-memory reservation admission. They exclude external persistence,
// account-balance holds, Holochain conductor work and provider settlement.
func BenchmarkCompensationReserveV2(b *testing.B) {
	book, authorization := benchmarkCompensationStateV2(b)
	b.ReportAllocs()
	b.ResetTimer()
	for index := uint64(0); index < uint64(b.N); index++ {
		request := benchmarkCompensationRequestV2(index, authorization)
		if _, err := book.Reserve(request, 1); err != nil { b.Fatal(err) }
	}
}

func BenchmarkCompensationReserveParallelV2(b *testing.B) {
	book, authorization := benchmarkCompensationStateV2(b)
	var next atomic.Uint64
	b.ReportAllocs()
	b.ResetTimer()
	b.RunParallel(func(pb *testing.PB) {
		for pb.Next() {
			index := next.Add(1)
			request := benchmarkCompensationRequestV2(index, authorization)
			if _, err := book.Reserve(request, 1); err != nil {
				b.Errorf("reserve compensation %d: %v", index, err)
				return
			}
		}
	})
}
