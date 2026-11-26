package conformance

import (
	"crypto/sha256"
	"errors"
)

type Hash32 [32]byte

const maxProofNodes = 65

func LeafHash(entry []byte) Hash32 {
	buf := make([]byte, 1+len(entry))
	buf[0] = 0
	copy(buf[1:], entry)
	return sha256.Sum256(buf)
}

func NodeHash(left, right Hash32) Hash32 {
	var buf [65]byte
	buf[0] = 1
	copy(buf[1:33], left[:])
	copy(buf[33:], right[:])
	return sha256.Sum256(buf[:])
}

func EmptyTreeHash() Hash32 {
	return sha256.Sum256(nil)
}

func largestPowerOfTwoLessThan(value int) int {
	if value <= 1 {
		panic("value must exceed one")
	}
	power := 1
	for power<<1 < value {
		power <<= 1
	}
	return power
}

func TreeHash(entries [][]byte) Hash32 {
	if len(entries) == 0 {
		return EmptyTreeHash()
	}
	leaves := make([]Hash32, len(entries))
	for i, entry := range entries {
		leaves[i] = LeafHash(entry)
	}
	return treeHashRange(leaves)
}

func treeHashRange(leaves []Hash32) Hash32 {
	switch len(leaves) {
	case 0:
		return EmptyTreeHash()
	case 1:
		return leaves[0]
	default:
		split := largestPowerOfTwoLessThan(len(leaves))
		left := treeHashRange(leaves[:split])
		right := treeHashRange(leaves[split:])
		return NodeHash(left, right)
	}
}

func inclusionPath(leaves []Hash32, index int, proof *[]Hash32) {
	if len(leaves) <= 1 {
		return
	}
	split := largestPowerOfTwoLessThan(len(leaves))
	if index < split {
		inclusionPath(leaves[:split], index, proof)
		*proof = append(*proof, treeHashRange(leaves[split:]))
	} else {
		inclusionPath(leaves[split:], index-split, proof)
		*proof = append(*proof, treeHashRange(leaves[:split]))
	}
}

func InclusionProof(entries [][]byte, index uint64) ([]Hash32, error) {
	if len(entries) == 0 {
		return nil, errors.New("empty tree")
	}
	if index >= uint64(len(entries)) {
		return nil, errors.New("leaf index out of range")
	}
	leaves := make([]Hash32, len(entries))
	for i, entry := range entries {
		leaves[i] = LeafHash(entry)
	}
	var proof []Hash32
	inclusionPath(leaves, int(index), &proof)
	return proof, nil
}

func VerifyInclusion(leaf Hash32, index, treeSize uint64, path []Hash32, root Hash32) error {
	if treeSize == 0 || index >= treeSize {
		return errors.New("invalid tree size or leaf index")
	}
	if len(path) > maxProofNodes {
		return errors.New("proof too long")
	}
	fn := index
	sn := treeSize - 1
	calculated := leaf
	for _, sibling := range path {
		if sn == 0 {
			return errors.New("extra proof node")
		}
		if (fn&1) == 1 || fn == sn {
			calculated = NodeHash(sibling, calculated)
			if fn&1 == 0 {
				for fn != 0 && fn&1 == 0 {
					fn >>= 1
					sn >>= 1
				}
			}
		} else {
			calculated = NodeHash(calculated, sibling)
		}
		fn >>= 1
		sn >>= 1
	}
	if sn != 0 {
		return errors.New("missing proof nodes")
	}
	if calculated != root {
		return errors.New("root mismatch")
	}
	return nil
}

func VerifyEntryInclusion(entry []byte, index, treeSize uint64, path []Hash32, root Hash32) error {
	return VerifyInclusion(LeafHash(entry), index, treeSize, path, root)
}

func consistencySubproof(firstSize int, leaves []Hash32, known bool, proof *[]Hash32) {
	if firstSize == len(leaves) {
		if !known {
			*proof = append(*proof, treeHashRange(leaves))
		}
		return
	}
	split := largestPowerOfTwoLessThan(len(leaves))
	if firstSize <= split {
		consistencySubproof(firstSize, leaves[:split], known, proof)
		*proof = append(*proof, treeHashRange(leaves[split:]))
	} else {
		consistencySubproof(firstSize-split, leaves[split:], false, proof)
		*proof = append(*proof, treeHashRange(leaves[:split]))
	}
}

func ConsistencyProof(entries [][]byte, firstSize uint64) ([]Hash32, error) {
	if firstSize > uint64(len(entries)) {
		return nil, errors.New("invalid consistency range")
	}
	if firstSize == 0 || firstSize == uint64(len(entries)) {
		return []Hash32{}, nil
	}
	leaves := make([]Hash32, len(entries))
	for i, entry := range entries {
		leaves[i] = LeafHash(entry)
	}
	var proof []Hash32
	consistencySubproof(int(firstSize), leaves, true, &proof)
	return proof, nil
}

func isPowerOfTwo(value uint64) bool {
	return value != 0 && value&(value-1) == 0
}

func VerifyConsistency(firstSize uint64, firstRoot Hash32, secondSize uint64, secondRoot Hash32, path []Hash32) error {
	if firstSize > secondSize || len(path) > maxProofNodes {
		return errors.New("invalid consistency range or proof length")
	}
	if firstSize == 0 {
		if len(path) == 0 && firstRoot == EmptyTreeHash() {
			return nil
		}
		return errors.New("invalid empty-tree consistency proof")
	}
	if firstSize == secondSize {
		if len(path) == 0 && firstRoot == secondRoot {
			return nil
		}
		return errors.New("invalid equal-size consistency proof")
	}
	if len(path) == 0 {
		return errors.New("missing consistency proof")
	}

	fn := firstSize - 1
	sn := secondSize - 1
	for fn&1 == 1 {
		fn >>= 1
		sn >>= 1
	}
	position := 0
	firstCalculated, secondCalculated := Hash32{}, Hash32{}
	if isPowerOfTwo(firstSize) {
		firstCalculated = firstRoot
		secondCalculated = firstRoot
	} else {
		firstCalculated = path[0]
		secondCalculated = path[0]
		position = 1
	}
	for ; position < len(path); position++ {
		node := path[position]
		if sn == 0 {
			return errors.New("extra consistency proof node")
		}
		if fn&1 == 1 || fn == sn {
			firstCalculated = NodeHash(node, firstCalculated)
			secondCalculated = NodeHash(node, secondCalculated)
			if fn&1 == 0 {
				for fn != 0 && fn&1 == 0 {
					fn >>= 1
					sn >>= 1
				}
			}
		} else {
			secondCalculated = NodeHash(secondCalculated, node)
		}
		fn >>= 1
		sn >>= 1
	}
	if sn != 0 || firstCalculated != firstRoot || secondCalculated != secondRoot {
		return errors.New("consistency proof root mismatch")
	}
	return nil
}
