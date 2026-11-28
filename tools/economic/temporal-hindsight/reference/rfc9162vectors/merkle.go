package rfc9162vectors

import (
	"crypto/sha256"
	"errors"
)

type Hash [32]byte

func LeafHash(entry []byte) Hash {
	input := make([]byte, 1+len(entry))
	input[0] = 0
	copy(input[1:], entry)
	return sha256.Sum256(input)
}

func NodeHash(left, right Hash) Hash {
	var input [65]byte
	input[0] = 1
	copy(input[1:33], left[:])
	copy(input[33:], right[:])
	return sha256.Sum256(input[:])
}

func EmptyTreeHash() Hash { return sha256.Sum256(nil) }

func largestPowerOfTwoLessThan(n int) int {
	k := 1
	for k <= (n-1)/2 {
		k <<= 1
	}
	return k
}

func MTH(leaves []Hash) Hash {
	switch len(leaves) {
	case 0:
		return EmptyTreeHash()
	case 1:
		return leaves[0]
	default:
		k := largestPowerOfTwoLessThan(len(leaves))
		return NodeHash(MTH(leaves[:k]), MTH(leaves[k:]))
	}
}

func VerifyInclusion(leaf Hash, index, size uint64, root Hash, path []Hash) error {
	if size == 0 { return errors.New("empty tree") }
	if index >= size { return errors.New("leaf index out of range") }
	if len(path) > 64 { return errors.New("proof too long") }
	fn, sn, calculated := index, size-1, leaf
	for _, node := range path {
		if sn == 0 { return errors.New("extra path node") }
		if fn&1 == 1 || fn == sn {
			calculated = NodeHash(node, calculated)
			if fn&1 == 0 {
				for fn&1 == 0 && fn != 0 { fn >>= 1; sn >>= 1 }
			}
		} else {
			calculated = NodeHash(calculated, node)
		}
		fn >>= 1
		sn >>= 1
	}
	if sn != 0 { return errors.New("truncated path") }
	if calculated != root { return errors.New("root mismatch") }
	return nil
}
