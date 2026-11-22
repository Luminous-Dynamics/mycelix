# Sensor observation → soil-indicator estimate lineage validator v0.1

**Status:** synthetic-only provenance and semantic validator. It does not read device files, calibrate spectra, fit a prediction model, evaluate scientific accuracy or replace wet chemistry.

## Why separate the records?

Hu et al. (2026), *On soil health and the pivotal role of sensing*, SOIL 12, 227–252, DOI [10.5194/soil-12-227-2026](https://doi.org/10.5194/soil-12-227-2026), distinguish direct sensor measurements from indirect estimates derived through relationships with other directly measured properties. Their review notes that standardized soil-health sensor protocols remain underdeveloped.

For spectroscopy, the instrument directly records an optical signal under a particular wavelength range, geometry, sample preparation and calibration state. A property such as SOC, pH, available P, CEC or biological activity inferred from that spectrum is a model-derived estimate. The physical sensor may be precise while the derived indicator is biased, out-of-domain or poorly calibrated.

## Three separate evidence records

1. **Sensor observation:** raw / primary signal, instrument instance, sensor modality, wavelength/frequency or acquisition configuration, sample identity, calibration artifact, acquisition context, QA, replicates, raw artifact URI, SHA-256, byte size and native support.
2. **Derived indicator estimate:** target property and unit, value/interval, model artifact digest, preprocessing digest, training sample and independence-group lineage, input-observation edges, applicability-domain status and explicit prediction semantics.
3. **Reference assay:** independent analytical/laboratory measurement, method, extraction basis, unit, depth, lab/batch, uncertainty and its declared role (calibration or independent validation).

## Graph relations

- sensor observation → acquired by → instrument instance;
- sensor observation → uses → calibration artifact;
- derived estimate → derived from → one or more sensor observation IDs;
- derived estimate → uses → model artifact and preprocessing artifact;
- model artifact → trained on → sample / independence groups;
- reference assay → measures → physical sample and target property;
- independent validation assay → validates → a frozen estimate/model only when its sample/group did not contribute to training or calibration.

## Fail-closed rules

- A raw spectral signal cannot be labelled as a direct measurement of SOC, available P, CEC or another unmeasured target property.
- A model-derived estimate cannot be relabelled as a laboratory reference assay.
- The sample and independence group for an independent validation estimate cannot overlap the model's training sets.
- An estimate must cite the actual sensor-observation IDs from the same physical sample.
- Model and preprocessing artifacts each require their own valid SHA-256.
- Calibration, sample preparation, instrument identity and acquisition conditions must be explicit.
- Out-of-domain estimates remain `abstain` and cannot be reported as qualified.
- A reference assay is preserved separately; it is never overwritten by the prediction.

## Synthetic fixture

The manufactured example represents a handheld NIR spectrum, an SOC estimate produced by a model from that spectrum, and a separate dry-combustion SOC reference assay from the same synthetic sample. Model training uses disjoint samples and independence groups. The fixture demonstrates data relationships only; it says nothing about actual NeoSpectra or Highveld model accuracy.

## Proposed qualification targets (not achieved)

- Zero direct-measurement promotion errors.
- Zero sample/group leakage in synthetic and real split audits.
- 100% lineage resolution for all accepted estimates.
- Explicit abstention for out-of-domain or missing calibration / preprocessing provenance.
- Every reported estimate binds to the exact sensor observation, model and preprocessing digests.
- Real-data follow-up must validate by whole sample, site, instrument, batch, survey and country as relevant; never split replicate scans independently.

## Boundary

This validator does not acquire or license real data, evaluate the quality of the calibration, estimate uncertainty calibration, or qualify any agronomic use. All values and identities in v0.1 are synthetic.