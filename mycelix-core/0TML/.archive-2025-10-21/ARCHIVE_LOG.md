# Archive Log - October 21, 2025

## Why These Files Were Archived

### baselines/pogq.py
- **Issue**: Imports non-existent `pogq_system` module
- **Status**: Cannot run, broken implementation
- **Replacement**: `src/zerotrustml/experimental/trust_layer.py`

### baselines/pogq_real.py
- **Issue**: Simplified mean-based PoGQ fails at 40% BFT (contamination)
- **Status**: Superseded by better implementation
- **Replacement**: `src/zerotrustml/experimental/trust_layer.py` ProofOfGradientQuality

## Investigation Summary

After thorough testing, discovered:
1. Real PoGQ exists in `trust_layer.py` (validates against test set)
2. Baseline tests used Multi-KRUM (not PoGQ) at 30% BFT
3. 40% BFT test failed with 37.5% detection rate
4. 30% BFT is the validated, honest approach for grant submission

## Next Steps

1. Use `trust_layer.py` for all PoGQ implementations
2. Test at 30% BFT to validate matching baseline results
3. Submit grant with validated 30% BFT claims
4. Document 40-50% as future work
