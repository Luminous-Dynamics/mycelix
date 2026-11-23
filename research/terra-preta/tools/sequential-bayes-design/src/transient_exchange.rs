//! Time-varying CT-informed macropore–matrix exchange reference.
//!
//! Implements q_exchange(t) = a_wetted(t) * j_macropore(t), the dimensional
//! product described by Chaif et al. (2025). Inputs are supplied explicitly;
//! this is not the full dual-permeability model and does not derive the inputs
//! from CT or solve Richards flow.
//!
//! Area and flux are linearly interpolated between samples. Their product is
//! integrated analytically on each interval.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExchangeObservation {
    pub time_seconds: f64,
    pub wetted_area_per_m: f64,
    pub macropore_flux_m_per_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExchangeSample {
    pub time_seconds: f64,
    pub wetted_area_per_m: f64,
    pub macropore_flux_m_per_s: f64,
    pub exchange_rate_per_s: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TransientExchangeSeries {
    samples: Vec<ExchangeSample>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExchangeBalanceCheck {
    pub integrated_transfer_fraction: f64,
    pub macropore_residual: f64,
    pub matrix_residual: f64,
    pub relative_max_residual: f64,
    pub balanced: bool,
}

impl TransientExchangeSeries {
    pub fn new(observations: &[ExchangeObservation]) -> Result<Self, String> {
        if observations.len() < 2 {
            return Err("at least two exchange observations are required".into());
        }
        let mut samples = Vec::with_capacity(observations.len());
        for (index, item) in observations.iter().enumerate() {
            if !item.time_seconds.is_finite()
                || !item.wetted_area_per_m.is_finite()
                || !item.macropore_flux_m_per_s.is_finite()
            {
                return Err("exchange inputs must be finite".into());
            }
            if item.wetted_area_per_m < 0.0 {
                return Err("wetted interfacial area must be non-negative".into());
            }
            if index > 0 && item.time_seconds <= observations[index - 1].time_seconds {
                return Err("exchange sample times must be strictly increasing".into());
            }
            let rate = item.wetted_area_per_m * item.macropore_flux_m_per_s;
            if !rate.is_finite() {
                return Err("exchange rate is non-finite".into());
            }
            samples.push(ExchangeSample {
                time_seconds: item.time_seconds,
                wetted_area_per_m: item.wetted_area_per_m,
                macropore_flux_m_per_s: item.macropore_flux_m_per_s,
                exchange_rate_per_s: rate,
            });
        }
        Ok(Self { samples })
    }

    pub fn samples(&self) -> &[ExchangeSample] { &self.samples }
    pub fn start_time(&self) -> f64 { self.samples[0].time_seconds }
    pub fn end_time(&self) -> f64 { self.samples[self.samples.len() - 1].time_seconds }

    fn interpolate(left: ExchangeSample, right: ExchangeSample, time: f64)
        -> Result<ExchangeSample, String>
    {
        let duration = right.time_seconds - left.time_seconds;
        if duration <= 0.0 || time < left.time_seconds || time > right.time_seconds {
            return Err("interpolation time is outside the selected segment".into());
        }
        let fraction = (time - left.time_seconds) / duration;
        let area = left.wetted_area_per_m
            + fraction * (right.wetted_area_per_m - left.wetted_area_per_m);
        let flux = left.macropore_flux_m_per_s
            + fraction * (right.macropore_flux_m_per_s - left.macropore_flux_m_per_s);
        let rate = area * flux;
        if !area.is_finite() || !flux.is_finite() || !rate.is_finite() || area < 0.0 {
            return Err("interpolated exchange state is invalid".into());
        }
        Ok(ExchangeSample {
            time_seconds: time,
            wetted_area_per_m: area,
            macropore_flux_m_per_s: flux,
            exchange_rate_per_s: rate,
        })
    }

    /// Evaluate the inputs and exchange rate within sampled support; no extrapolation.
    pub fn sample_at(&self, time: f64) -> Result<ExchangeSample, String> {
        if !time.is_finite() || time < self.start_time() || time > self.end_time() {
            return Err("requested time is outside exchange sample support".into());
        }
        if time == self.end_time() {
            return Ok(self.samples[self.samples.len() - 1]);
        }
        let segment = self.samples.windows(2)
            .find(|pair| time >= pair[0].time_seconds && time <= pair[1].time_seconds)
            .ok_or_else(|| "no exchange interpolation segment contains time".to_string())?;
        Self::interpolate(segment[0], segment[1], time)
    }

    /// Exact integral for piecewise-linear area and flux input signals.
    pub fn integrate_between(&self, start: f64, end: f64) -> Result<f64, String> {
        if !start.is_finite() || !end.is_finite() || end <= start {
            return Err("integration requires finite end time greater than start".into());
        }
        if start < self.start_time() || end > self.end_time() {
            return Err("integration interval exceeds sampled support".into());
        }
        let mut times = vec![start];
        times.extend(self.samples.iter().map(|s| s.time_seconds)
            .filter(|time| *time > start && *time < end));
        times.push(end);
        let mut total = 0.0;
        for pair in times.windows(2) {
            let left = self.sample_at(pair[0])?;
            let right = self.sample_at(pair[1])?;
            let dt = pair[1] - pair[0];
            let da = right.wetted_area_per_m - left.wetted_area_per_m;
            let dj = right.macropore_flux_m_per_s - left.macropore_flux_m_per_s;
            // Integral_0^1 (a0 + da*u)(j0 + dj*u) du, multiplied by dt.
            total += dt * (
                left.wetted_area_per_m * left.macropore_flux_m_per_s
                + 0.5 * (
                    left.wetted_area_per_m * dj
                    + left.macropore_flux_m_per_s * da
                )
                + da * dj / 3.0
            );
        }
        if total.is_finite() { Ok(total) } else { Err("integrated exchange is non-finite".into()) }
    }

    pub fn integrate_full(&self) -> Result<f64, String> {
        self.integrate_between(self.start_time(), self.end_time())
    }
}

/// Check paired compartment changes for the isolated exchange contribution.
/// Positive transfer means macropore loss and matrix gain. Other inputs,
/// drainage, evaporation and uptake must be accounted for separately.
pub fn check_paired_exchange_balance(
    macropore_storage_change: f64,
    matrix_storage_change: f64,
    integrated_transfer: f64,
    relative_tolerance: f64,
) -> Result<ExchangeBalanceCheck, String> {
    if [macropore_storage_change, matrix_storage_change,
        integrated_transfer, relative_tolerance].iter().any(|x| !x.is_finite())
    {
        return Err("exchange balance inputs must be finite".into());
    }
    if relative_tolerance < 0.0 {
        return Err("relative tolerance must be non-negative".into());
    }
    let macro_residual = macropore_storage_change + integrated_transfer;
    let matrix_residual = matrix_storage_change - integrated_transfer;
    let scale = macropore_storage_change.abs()
        .max(matrix_storage_change.abs())
        .max(integrated_transfer.abs())
        .max(1e-12);
    let relative = macro_residual.abs().max(matrix_residual.abs()) / scale;
    Ok(ExchangeBalanceCheck {
        integrated_transfer_fraction: integrated_transfer,
        macropore_residual: macro_residual,
        matrix_residual,
        relative_max_residual: relative,
        balanced: relative <= relative_tolerance,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn p(t: f64, a: f64, j: f64) -> ExchangeObservation {
        ExchangeObservation { time_seconds: t, wetted_area_per_m: a, macropore_flux_m_per_s: j }
    }

    #[test]
    fn constant_inputs_integrate_exactly() {
        let s = TransientExchangeSeries::new(&[p(0.0, 2.0, 0.01), p(10.0, 2.0, 0.01)]).unwrap();
        assert!((s.sample_at(5.0).unwrap().exchange_rate_per_s - 0.02).abs() < 1e-12);
        assert!((s.integrate_full().unwrap() - 0.2).abs() < 1e-12);
    }

    #[test]
    fn product_of_linear_inputs_has_exact_quadratic_integral() {
        let s = TransientExchangeSeries::new(&[p(2.0, 1.0, 2.0), p(5.0, 3.0, 4.0)]).unwrap();
        assert!((s.integrate_full().unwrap() - 19.0).abs() < 1e-12);
    }

    #[test]
    fn partial_irregular_interval_uses_interpolated_area_and_flux() {
        let s = TransientExchangeSeries::new(&[
            p(0.0, 1.0, 0.01), p(3.0, 4.0, 0.04), p(8.0, 2.0, 0.02),
        ]).unwrap();
        assert!((s.sample_at(1.0).unwrap().wetted_area_per_m - 2.0).abs() < 1e-12);
        assert!((s.sample_at(1.0).unwrap().macropore_flux_m_per_s - 0.02).abs() < 1e-12);
        assert!(s.integrate_between(1.0, 6.0).unwrap() > 0.0);
        assert!(s.integrate_between(6.0, 1.0).is_err());
    }

    #[test]
    fn synthetic_rainfall_onset_exchange_decays_to_zero() {
        let s = TransientExchangeSeries::new(&[
            p(0.0, 4.0, 0.03), p(30.0, 3.0, 0.02),
            p(60.0, 2.0, 0.005), p(90.0, 1.0, 0.0),
        ]).unwrap();
        assert!(s.samples()[0].exchange_rate_per_s > s.samples()[2].exchange_rate_per_s);
        assert_eq!(s.samples()[3].exchange_rate_per_s, 0.0);
    }

    #[test]
    fn negative_flux_represents_reverse_transfer() {
        let s = TransientExchangeSeries::new(&[p(0.0, 1.0, -0.01), p(10.0, 1.0, -0.01)]).unwrap();
        assert!((s.integrate_full().unwrap() + 0.1).abs() < 1e-12);
    }

    #[test]
    fn rejects_invalid_time_area_flux_and_extrapolation() {
        assert!(TransientExchangeSeries::new(&[p(0.0, 1.0, 0.1)]).is_err());
        assert!(TransientExchangeSeries::new(&[p(0.0, 1.0, 0.1), p(0.0, 2.0, 0.1)]).is_err());
        assert!(TransientExchangeSeries::new(&[p(0.0, -1.0, 0.1), p(1.0, 1.0, 0.1)]).is_err());
        assert!(TransientExchangeSeries::new(&[p(0.0, 1.0, f64::NAN), p(1.0, 1.0, 0.1)]).is_err());
        let s = TransientExchangeSeries::new(&[p(0.0, 1.0, 0.1), p(1.0, 1.0, 0.1)]).unwrap();
        assert!(s.sample_at(-0.1).is_err());
        assert!(s.integrate_between(0.0, 2.0).is_err());
    }

    #[test]
    fn paired_exchange_storage_balance_closes_both_compartments() {
        let ok = check_paired_exchange_balance(-0.2, 0.2, 0.2, 1e-12).unwrap();
        assert!(ok.balanced);
        let bad = check_paired_exchange_balance(-0.2, 0.15, 0.2, 1e-6).unwrap();
        assert!(!bad.balanced);
        assert!(check_paired_exchange_balance(-0.2, 0.2, 0.2, -1e-6).is_err());
    }
}
