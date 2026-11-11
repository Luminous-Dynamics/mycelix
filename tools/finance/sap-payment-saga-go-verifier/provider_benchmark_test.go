package sagaverifier

import "testing"

func providerBenchmarkFixtureV1(b *testing.B) (ProviderContractV1, ProviderOperationKeyV1, ProviderRetryPermitV1) {
	b.Helper()
	contract := AdyenDocumentationBaselineV1("company:benchmark-account")
	contractDigest, err := contract.CanonicalDigest()
	if err != nil { b.Fatal(err) }
	key := ProviderOperationKeyV1{
		ProviderID: contract.ProviderID,
		ProfileID: contract.ProfileID,
		APIVersion: contract.APIVersion,
		AccountScope: contract.AccountScope,
		Region: "eu",
		IdempotencyKey: "idem-00000000-0000-4000-8000-000000000001",
		RequestDigest: providerTestDigestV1('a'),
		CreatedAtUnixMS: 1_000,
		ContractDigest: contractDigest,
	}
	permit, err := key.QualifyRetry(
		contract, key.AccountScope, key.Region, key.RequestDigest,
		ProviderTransportTimeoutUnknownV1, 2_000,
	)
	if err != nil { b.Fatal(err) }
	return contract, key, permit
}

func BenchmarkProviderKeyValidateV1(b *testing.B) {
	contract, key, _ := providerBenchmarkFixtureV1(b)
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if err := key.Validate(contract); err != nil { b.Fatal(err) }
	}
}

func BenchmarkProviderRetryPermitQualifyV1(b *testing.B) {
	contract, key, _ := providerBenchmarkFixtureV1(b)
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if _, err := key.QualifyRetry(
			contract, key.AccountScope, key.Region, key.RequestDigest,
			ProviderTransportTimeoutUnknownV1, 2_000,
		); err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkProviderRetryPermitRevalidateV1(b *testing.B) {
	contract, key, permit := providerBenchmarkFixtureV1(b)
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if err := permit.ValidateAgainst(key, contract, 2_000); err != nil { b.Fatal(err) }
	}
}

func BenchmarkProviderRetryPermitQualifyParallelV1(b *testing.B) {
	contract, key, _ := providerBenchmarkFixtureV1(b)
	b.ReportAllocs()
	b.ResetTimer()
	b.RunParallel(func(pb *testing.PB) {
		for pb.Next() {
			if _, err := key.QualifyRetry(
				contract, key.AccountScope, key.Region, key.RequestDigest,
				ProviderTransportTimeoutUnknownV1, 2_000,
			); err != nil {
				b.Errorf("qualify retry: %v", err)
				return
			}
		}
	})
}
