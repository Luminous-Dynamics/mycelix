package sagaverifier

import (
	"os"
	"strings"
	"testing"
)

const stripeWebhookTestSecretV1 = "whsec_test_secret_2026"
const stripeWebhookTestTimestampV1 uint64 = 1_794_000_000
const stripeWebhookTestBodyV1 = "{\"id\":\"evt_test_123\",\"object\":\"event\",\"type\":\"refund.updated\"}"
const stripeWebhookTestSignatureV1 = "7379bbb016143ced7ebf6f8083e935c452f08f6c7d3716de1715a9716ea21912"

func stripeWebhookKATV1(t *testing.T) map[string]string {
	t.Helper()
	raw, err := os.ReadFile("../sap-payment-saga-model/fixtures/sap-provider-contract-v1.kat")
	if err != nil {
		t.Fatal(err)
	}
	values := map[string]string{}
	for _, line := range strings.Split(string(raw), "\n") {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		fields := strings.SplitN(line, "=", 2)
		if len(fields) != 2 {
			t.Fatalf("invalid provider KAT line: %q", line)
		}
		values[fields[0]] = fields[1]
	}
	return values
}

func TestStripeWebhookSharedKnownAnswerV1(t *testing.T) {
	kat := stripeWebhookKATV1(t)
	if kat["stripe_webhook_timestamp"] != "1794000000" {
		t.Fatalf("timestamp KAT=%s", kat["stripe_webhook_timestamp"])
	}
	if kat["stripe_webhook_signature_v1"] != stripeWebhookTestSignatureV1 {
		t.Fatal("signature KAT differs")
	}
	header := "t=" + kat["stripe_webhook_timestamp"] + ",v1=" + kat["stripe_webhook_signature_v1"]
	verified, err := VerifyStripeWebhookV1(
		[]byte(stripeWebhookTestBodyV1), header, []byte(stripeWebhookTestSecretV1),
		stripeWebhookTestTimestampV1+10, 300,
	)
	if err != nil {
		t.Fatal(err)
	}
	if verified.RawBodySHA256 != kat["stripe_webhook_raw_body_sha256"] {
		t.Fatalf("body digest=%s want=%s", verified.RawBodySHA256, kat["stripe_webhook_raw_body_sha256"])
	}
	if verified.TimestampUnixSeconds != stripeWebhookTestTimestampV1 || verified.MatchingV1SignatureCount != 1 {
		t.Fatalf("unexpected verification result: %+v", verified)
	}
}

func TestStripeWebhookRotatedSignaturesAndUnknownHeaderFieldsV1(t *testing.T) {
	header := "v0=legacy,t=1794000000,v1=" + strings.Repeat("00", 32) +
		",future=metadata,v1=" + stripeWebhookTestSignatureV1
	verified, err := VerifyStripeWebhookV1(
		[]byte(stripeWebhookTestBodyV1), header, []byte(stripeWebhookTestSecretV1),
		stripeWebhookTestTimestampV1, 300,
	)
	if err != nil {
		t.Fatal(err)
	}
	if verified.MatchingV1SignatureCount != 1 {
		t.Fatalf("matches=%d", verified.MatchingV1SignatureCount)
	}
}

func TestStripeWebhookHeaderOrderingAndWhitespaceV1(t *testing.T) {
	header := " v1=" + stripeWebhookTestSignatureV1 + " , t=1794000000 "
	if _, err := VerifyStripeWebhookV1(
		[]byte(stripeWebhookTestBodyV1), header, []byte(stripeWebhookTestSecretV1),
		stripeWebhookTestTimestampV1, 300,
	); err != nil {
		t.Fatal(err)
	}
}

func TestStripeWebhookModifiedRawBytesAndWrongSecretFailV1(t *testing.T) {
	header := "t=1794000000,v1=" + stripeWebhookTestSignatureV1
	changed := strings.ReplaceAll(stripeWebhookTestBodyV1, "refund.updated", "refund.failed")
	if _, err := VerifyStripeWebhookV1([]byte(changed), header, []byte(stripeWebhookTestSecretV1), stripeWebhookTestTimestampV1, 300); err != ErrStripeWebhookSignatureMismatchV1 {
		t.Fatalf("modified body error=%v", err)
	}
	if _, err := VerifyStripeWebhookV1([]byte(stripeWebhookTestBodyV1), header, []byte("wrong-secret"), stripeWebhookTestTimestampV1, 300); err != ErrStripeWebhookSignatureMismatchV1 {
		t.Fatalf("wrong secret error=%v", err)
	}
}

func TestStripeWebhookStaleFutureAndToleranceBoundaryV1(t *testing.T) {
	header := "t=1794000000,v1=" + stripeWebhookTestSignatureV1
	if _, err := VerifyStripeWebhookV1([]byte(stripeWebhookTestBodyV1), header, []byte(stripeWebhookTestSecretV1), stripeWebhookTestTimestampV1+301, 300); err != ErrStripeWebhookTimestampOutsideToleranceV1 {
		t.Fatalf("stale timestamp error=%v", err)
	}
	if _, err := VerifyStripeWebhookV1([]byte(stripeWebhookTestBodyV1), header, []byte(stripeWebhookTestSecretV1), stripeWebhookTestTimestampV1-301, 300); err != ErrStripeWebhookTimestampOutsideToleranceV1 {
		t.Fatalf("future timestamp error=%v", err)
	}
	if _, err := VerifyStripeWebhookV1([]byte(stripeWebhookTestBodyV1), header, []byte(stripeWebhookTestSecretV1), stripeWebhookTestTimestampV1+300, 300); err != nil {
		t.Fatalf("inclusive boundary rejected: %v", err)
	}
}

func TestStripeWebhookMalformedHeadersFailClosedV1(t *testing.T) {
	cases := []struct {
		header string
		want   error
	}{
		{"", ErrStripeWebhookEmptyHeaderV1},
		{"t=1794000000,t=1794000000,v1=" + stripeWebhookTestSignatureV1, ErrStripeWebhookDuplicateTimestampV1},
		{"v1=" + stripeWebhookTestSignatureV1, ErrStripeWebhookMissingTimestampV1},
		{"t=1794000000,v0=" + stripeWebhookTestSignatureV1, ErrStripeWebhookMissingV1SignatureV1},
		{"t=1794000000,v1=not-hex", ErrStripeWebhookInvalidSignatureEncodingV1},
		{"t=x,v1=" + stripeWebhookTestSignatureV1, ErrStripeWebhookInvalidTimestampV1},
		{"t=1794000000,v1=", ErrStripeWebhookInvalidHeaderFieldV1},
	}
	for _, tc := range cases {
		if _, err := VerifyStripeWebhookV1([]byte(stripeWebhookTestBodyV1), tc.header, []byte(stripeWebhookTestSecretV1), stripeWebhookTestTimestampV1, 300); err != tc.want {
			t.Fatalf("header %q error=%v want=%v", tc.header, err, tc.want)
		}
	}
}

func TestStripeWebhookEmptySecretNonASCIIAndOversizedHeaderFailV1(t *testing.T) {
	header := "t=1794000000,v1=" + stripeWebhookTestSignatureV1
	if _, err := VerifyStripeWebhookV1([]byte(stripeWebhookTestBodyV1), header, nil, stripeWebhookTestTimestampV1, 300); err != ErrStripeWebhookEmptySecretV1 {
		t.Fatalf("empty secret error=%v", err)
	}
	if _, err := VerifyStripeWebhookV1([]byte(stripeWebhookTestBodyV1), "t=1794000000,v1=é", []byte(stripeWebhookTestSecretV1), stripeWebhookTestTimestampV1, 300); err != ErrStripeWebhookInvalidHeaderEncodingV1 {
		t.Fatalf("non-ASCII header error=%v", err)
	}
	if _, err := VerifyStripeWebhookV1([]byte(stripeWebhookTestBodyV1), strings.Repeat("x", maxStripeSignatureHeaderBytesV1+1), []byte(stripeWebhookTestSecretV1), stripeWebhookTestTimestampV1, 300); err != ErrStripeWebhookHeaderTooLargeV1 {
		t.Fatalf("oversized header error=%v", err)
	}
}

func TestStripeWebhookZeroToleranceFailsClosedV1(t *testing.T) {
	header := "t=1794000000,v1=" + stripeWebhookTestSignatureV1
	if _, err := VerifyStripeWebhookV1(
		[]byte(stripeWebhookTestBodyV1), header, []byte(stripeWebhookTestSecretV1),
		stripeWebhookTestTimestampV1, 0,
	); err != ErrStripeWebhookInvalidToleranceV1 {
		t.Fatalf("zero tolerance error=%v", err)
	}
}

func TestStripeWebhookSecretRotationAcceptsAnyActiveSecretV1(t *testing.T) {
	header := "t=1794000000,v1=" + stripeWebhookTestSignatureV1
	secrets := [][]byte{[]byte("old-secret"), []byte(stripeWebhookTestSecretV1)}
	if _, err := VerifyStripeWebhookWithSecretsV1(
		[]byte(stripeWebhookTestBodyV1), header, secrets,
		stripeWebhookTestTimestampV1+1, StripeDefaultToleranceSecondsV1,
	); err != nil {
		t.Fatalf("active secret rotation rejected: %v", err)
	}
	if _, err := VerifyStripeWebhookWithSecretsV1(
		[]byte(stripeWebhookTestBodyV1), header, [][]byte{[]byte("old-secret")},
		stripeWebhookTestTimestampV1+1, StripeDefaultToleranceSecondsV1,
	); err != ErrStripeWebhookSignatureMismatchV1 {
		t.Fatalf("no matching secret error=%v", err)
	}
}

func TestStripeWebhookSharedKATPinsBodySignatureAndTimestampV1(t *testing.T) {
	kat := stripeWebhookKATV1(t)
	if kat["stripe_webhook_timestamp"] != "1794000000" {
		t.Fatalf("timestamp KAT=%s", kat["stripe_webhook_timestamp"])
	}
	if kat["stripe_webhook_signature_v1"] != stripeWebhookTestSignatureV1 {
		t.Fatal("signature KAT differs")
	}
	verified, err := VerifyStripeWebhookV1(
		[]byte(stripeWebhookTestBodyV1),
		"t="+kat["stripe_webhook_timestamp"]+",v1="+kat["stripe_webhook_signature_v1"],
		[]byte(stripeWebhookTestSecretV1), stripeWebhookTestTimestampV1+10,
		StripeDefaultToleranceSecondsV1,
	)
	if err != nil {
		t.Fatal(err)
	}
	if verified.RawBodySHA256 != kat["stripe_webhook_raw_body_sha256"] {
		t.Fatalf("body digest=%s want=%s", verified.RawBodySHA256, kat["stripe_webhook_raw_body_sha256"])
	}
}
