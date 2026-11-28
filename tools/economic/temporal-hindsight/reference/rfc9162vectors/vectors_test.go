package rfc9162vectors

import (
	"bufio"
	"encoding/hex"
	"os"
	"strconv"
	"strings"
	"testing"
)

const vectorFile = "../../../../../tools/economic/temporal-frontier-model/RFC9162_VECTORS_V1.txt"
const xeniaCheckpointVectorFile = "../../../../../tools/economic/temporal-frontier-model/XENIA_CHECKPOINT_V1_KAT.txt"

func decodeHash(t *testing.T, value string) Hash {
	t.Helper()
	var result Hash
	decoded, err := hex.DecodeString(value)
	if err != nil || len(decoded) != len(result) { t.Fatalf("invalid hash %q: %v", value, err) }
	copy(result[:], decoded)
	return result
}

func TestSharedRFC9162Vectors(t *testing.T) {
	file, err := os.Open(vectorFile)
	if err != nil { t.Fatal(err) }
	defer file.Close()

	leaves := map[string]Hash{}
	mthVectors := map[int]Hash{}
	type inclusionVector struct { name string; index, size uint64; root Hash; path []Hash }
	var inclusionVectors []inclusionVector

	scanner := bufio.NewScanner(file)
	line := 0
	for scanner.Scan() {
		line++
		text := strings.TrimSpace(scanner.Text())
		if text == "" || strings.HasPrefix(text, "#") || strings.HasPrefix(text, "MYCELIX-") { continue }
		fields := strings.Split(text, "|")
		switch fields[0] {
		case "leaf":
			if len(fields) != 4 { t.Fatalf("line %d: invalid leaf vector", line) }
			payload, err := hex.DecodeString(fields[2]); if err != nil { t.Fatal(err) }
			leaves[fields[1]] = decodeHash(t, fields[3])
			if got := LeafHash(payload); got != leaves[fields[1]] { t.Fatalf("line %d: leaf hash mismatch", line) }
		case "mth":
			if len(fields) != 3 { t.Fatalf("line %d: invalid mth vector", line) }
			size, err := strconv.Atoi(fields[1]); if err != nil { t.Fatal(err) }
			mthVectors[size] = decodeHash(t, fields[2])
		case "include":
			if len(fields) != 6 { t.Fatalf("line %d: invalid inclusion vector", line) }
			index, _ := strconv.ParseUint(fields[2],10,64)
			size, _ := strconv.ParseUint(fields[3],10,64)
			var path []Hash
			if fields[5] != "" {
				for _, item := range strings.Split(fields[5], ",") { path = append(path, decodeHash(t,item)) }
			}
			inclusionVectors = append(inclusionVectors,inclusionVector{fields[1],index,size,decodeHash(t,fields[4]),path})
		default:
			t.Fatalf("line %d: unknown vector type %q",line,fields[0])
		}
	}
	if err := scanner.Err(); err != nil { t.Fatal(err) }
	if len(leaves) != 3 { t.Fatalf("expected three shared leaves, got %d",len(leaves)) }
	ordered := []Hash{leaves["a"],leaves["b"],leaves["c"]}
	for size, expected := range mthVectors {
		if got := MTH(ordered[:size]); got != expected { t.Fatalf("MTH(%d) mismatch",size) }
	}
	for _, vector := range inclusionVectors {
		if err := VerifyInclusion(leaves[vector.name],vector.index,vector.size,vector.root,vector.path); err != nil {
			t.Fatalf("inclusion %s at %d: %v",vector.name,vector.index,err)
		}
	}
}

func TestXeniaCheckpointV1PayloadHasTheSameRFC9162LeafHash(t *testing.T) {
	content, err := os.ReadFile(xeniaCheckpointVectorFile)
	if err != nil {
		t.Fatal(err)
	}
	var payloadHex string
	var expectedLeafHex string
	var payloadLength int
	for _, line := range strings.Split(string(content), "\n") {
		fields := strings.SplitN(strings.TrimSpace(line), "=", 2)
		if len(fields) != 2 {
			continue
		}
		switch fields[0] {
		case "payload_len":
			payloadLength, err = strconv.Atoi(fields[1])
			if err != nil {
				t.Fatal(err)
			}
		case "payload_hex":
			payloadHex = fields[1]
		case "leaf_hash_sha256":
			expectedLeafHex = fields[1]
		}
	}
	if payloadHex == "" || expectedLeafHex == "" || payloadLength == 0 {
		t.Fatal("Xenia checkpoint vector is incomplete")
	}
	payload, err := hex.DecodeString(payloadHex)
	if err != nil {
		t.Fatal(err)
	}
	if len(payload) != payloadLength {
		t.Fatalf("Xenia payload length = %d, want %d", len(payload), payloadLength)
	}
	if got, want := LeafHash(payload), decodeHash(t, expectedLeafHex); got != want {
		t.Fatalf("Xenia checkpoint leaf hash = %x, want %x", got, want)
	}
}

func TestInclusionMutationNegativeControls(t *testing.T) {
	leafA := LeafHash([]byte("a"))
	leafB := LeafHash([]byte("b"))
	leafC := LeafHash([]byte("c"))
	root := NodeHash(NodeHash(leafA,leafB),leafC)
	if err := VerifyInclusion(leafA,0,3,root,[]Hash{leafB,leafC}); err != nil { t.Fatal(err) }
	if err := VerifyInclusion(LeafHash([]byte("wrong")),0,3,root,[]Hash{leafB,leafC}); err == nil { t.Fatal("mutated leaf accepted") }
	if err := VerifyInclusion(leafA,0,3,root,[]Hash{leafB}); err == nil { t.Fatal("truncated path accepted") }
}
