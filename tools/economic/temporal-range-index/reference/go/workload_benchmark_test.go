package temporalrange

import (
	"fmt"
	"testing"
)

func intervalBenchmarkWorkloadV1(name string, size int) ([]IntervalRecordV1, IntervalInstantV1) {
	records := make([]IntervalRecordV1, size)
	query := IntervalInstantV1{}
	for i := 0; i < size; i++ {
		var start, end int64
		open := false
		switch name {
		case "disjoint_sparse":
			start = int64(i * 4)
			end = start + 3
			query = intervalPointV1(int64(size*2), 0)
		case "clustered":
			start = int64(i/32) * 100
			end = start + 50
			query = intervalPointV1(int64((size/64)*100+25), 0)
		case "nested_all_overlap":
			start = int64(i)
			end = int64(2*size - i)
			query = intervalPointV1(int64(size), 0)
		case "all_overlap":
			start = 0
			end = int64(size * 2)
			query = intervalPointV1(int64(size), 0)
		case "future_only":
			start = int64(i + 1)
			end = start + 10
			query = intervalPointV1(0, 0)
		case "expired_only":
			start = int64(i - 2*size - 1)
			end = start + 1
			query = intervalPointV1(0, 0)
		case "open_ended":
			start = int64(i)
			open = true
			query = intervalPointV1(int64(size), 0)
		default:
			panic("unknown interval benchmark workload: " + name)
		}

		endBound := intervalFiniteV1(end, 0)
		if open {
			endBound = intervalOpenV1()
		}
		records[i] = IntervalRecordV1{
			AssertionRef: fmt.Sprintf("assertion:%s:%08d", name, i),
			ValidFrom: intervalPointV1(start, 0),
			ValidTo: endBound,
			ReceiptDigest: fmt.Sprintf("sha256:%064x", uint64(i+1)),
		}
	}
	return records, query
}

func intervalProofStatsV1(node *IntervalProofNodeV1) (nodes, pruned, branches, leaves int) {
	if node == nil {
		return 0, 0, 0, 0
	}
	nodes = 1
	switch node.Kind {
	case IntervalProofPrunedV1:
		return nodes, 1, 0, 0
	case IntervalProofLeafV1:
		return nodes, 0, 0, 1
	case IntervalProofBranchV1:
		leftN, leftP, leftB, leftL := intervalProofStatsV1(node.Left)
		rightN, rightP, rightB, rightL := intervalProofStatsV1(node.Right)
		return nodes + leftN + rightN, leftP + rightP, 1 + leftB + rightB, leftL + rightL
	default:
		return nodes, 0, 0, 0
	}
}

func BenchmarkTemporalQueryWorkloadsV1(b *testing.B) {
	workloads := []string{
		"disjoint_sparse",
		"clustered",
		"nested_all_overlap",
		"all_overlap",
		"future_only",
		"expired_only",
		"open_ended",
	}
	for _, size := range []int{1_000, 10_000, 100_000} {
		for _, workload := range workloads {
			records, query := intervalBenchmarkWorkloadV1(workload, size)
			b.Run(fmt.Sprintf("build/%s/%d", workload, size), func(b *testing.B) {
				scope := intervalScopeFixtureV1()
				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					if _, err := NewIntervalTreeV1(scope, records); err != nil {
						b.Fatal(err)
					}
				}
			})

			scope := intervalScopeFixtureV1()
			tree, err := NewIntervalTreeV1(scope, records)
			if err != nil {
				b.Fatal(err)
			}
			proof, err := tree.ProvePointQueryV1(query)
			if err != nil {
				b.Fatal(err)
			}
			verified, err := VerifyIntervalPointProofV1(scope, query, proof, tree.RootHash())
			if err != nil {
				b.Fatal(err)
			}
			encoded, err := MarshalIntervalPointProofV1(proof)
			if err != nil {
				b.Fatal(err)
			}
			nodes, pruned, branches, leaves := intervalProofStatsV1(proof.Tree)

			b.Run(fmt.Sprintf("linear_scan/%s/%d", workload, size), func(b *testing.B) {
				b.ReportMetric(float64(len(verified.Records)), "matches")
				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					matches := 0
					for _, record := range records {
						if intervalMatchesV1(record, query) {
							matches++
						}
					}
					if matches != len(verified.Records) {
						b.Fatalf("linear scan found %d matches; proof found %d", matches, len(verified.Records))
					}
				}
			})
			b.Run(fmt.Sprintf("proof_generate/%s/%d", workload, size), func(b *testing.B) {
				b.ReportMetric(float64(len(verified.Records)), "matches")
				b.ReportMetric(float64(nodes), "proof_nodes")
				b.ReportMetric(float64(pruned), "pruned_nodes")
				b.ReportMetric(float64(branches), "expanded_branches")
				b.ReportMetric(float64(leaves), "returned_leaves")
				b.ReportMetric(float64(len(encoded)), "proof_B")
				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					if _, err := tree.ProvePointQueryV1(query); err != nil {
						b.Fatal(err)
					}
				}
			})
			b.Run(fmt.Sprintf("proof_wire_decode_verify/%s/%d", workload, size), func(b *testing.B) {
				b.ReportMetric(float64(len(verified.Records)), "matches")
				b.ReportMetric(float64(nodes), "proof_nodes")
				b.ReportMetric(float64(len(encoded)), "proof_B")
				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					decoded, err := UnmarshalIntervalPointProofV1(encoded)
					if err != nil {
						b.Fatal(err)
					}
					result, err := VerifyIntervalPointProofV1(scope, query, decoded, tree.RootHash())
					if err != nil {
						b.Fatal(err)
					}
					if len(result.Records) != len(verified.Records) {
						b.Fatal("wire verification result cardinality changed")
					}
				}
			})
		}
	}
}
