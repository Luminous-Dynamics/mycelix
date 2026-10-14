#!/usr/bin/env python3
"""Independent conditional-normal replay of the correlated Gaussian disk fixture.

This uses a one-dimensional conditional-CDF integral, not the polar-area
quadrature implemented by verify_pc_correlated_gaussian_reference.py.
Requires the fixture's pinned NumPy dependency for Gauss-Legendre nodes.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
from numpy.polynomial.legendre import leggauss


FIXTURE = (
    Path(__file__).resolve().parents[1]
    / "tests"
    / "fixtures"
    / "pc_correlated_gaussian_disk_v1.json"
)
ORDERS = (500, 700)
MAX_RELATIVE_REFERENCE_ERROR = 5e-9
MAX_RELATIVE_CROSS_ORDER_DELTA = 5e-9


def integrate_by_conditional_normal(case: dict[str, Any], order: int) -> float:
    """Integrate disk mass using f_X(x) * P(-ymax <= Y <= ymax | X=x)."""
    radius = float(case["hard_body_radius_km"])
    mean = np.asarray(case["miss_encounter_plane_km"], dtype=np.float64)
    covariance = np.asarray(
        case["covariance_encounter_plane_km2"], dtype=np.float64
    )

    if mean.shape != (2,) or covariance.shape != (2, 2):
        raise ValueError(f"{case['case_id']}: invalid vector/matrix shape")
    if not np.isfinite(mean).all() or not np.isfinite(covariance).all():
        raise ValueError(f"{case['case_id']}: non-finite input")
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError(f"{case['case_id']}: radius must be finite and positive")
    if not np.allclose(covariance, covariance.T, rtol=1e-12, atol=0.0):
        raise ValueError(f"{case['case_id']}: covariance is not symmetric")

    variance_x = float(covariance[0, 0])
    covariance_xy = float(covariance[0, 1])
    variance_y = float(covariance[1, 1])
    conditional_variance_y = variance_y - covariance_xy**2 / variance_x
    determinant = variance_x * variance_y - covariance_xy**2
    if (
        not math.isfinite(variance_x)
        or variance_x <= 0
        or not math.isfinite(conditional_variance_y)
        or conditional_variance_y <= 0
        or not math.isfinite(determinant)
        or determinant <= 0
    ):
        raise ValueError(f"{case['case_id']}: covariance must be positive definite")

    sigma_x = math.sqrt(variance_x)
    sigma_conditional_y = math.sqrt(conditional_variance_y)
    nodes, weights = leggauss(order)
    xs = radius * nodes
    weighted_dx = radius * weights
    inv_sqrt_two = 1.0 / math.sqrt(2.0)
    sqrt_two_pi = math.sqrt(2.0 * math.pi)

    conditional_masses = []
    for x in xs:
        x_value = float(x)
        y_limit = math.sqrt(max(0.0, radius * radius - x_value * x_value))
        conditional_mean_y = float(mean[1]) + covariance_xy / variance_x * (
            x_value - float(mean[0])
        )
        upper_z = (y_limit - conditional_mean_y) / sigma_conditional_y
        lower_z = (-y_limit - conditional_mean_y) / sigma_conditional_y
        conditional_mass = 0.5 * (
            math.erf(upper_z * inv_sqrt_two)
            - math.erf(lower_z * inv_sqrt_two)
        )
        standardized_x = (x_value - float(mean[0])) / sigma_x
        marginal_density_x = math.exp(-0.5 * standardized_x**2) / (
            sigma_x * sqrt_two_pi
        )
        conditional_masses.append(marginal_density_x * conditional_mass)

    return float(
        np.dot(weighted_dx, np.asarray(conditional_masses, dtype=np.float64))
    )


def main() -> int:
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    failures: list[str] = []
    output: list[dict[str, Any]] = []

    for case in fixture["cases"]:
        values = [integrate_by_conditional_normal(case, order) for order in ORDERS]
        expected = float(case["expected_pc_integral_reference"])
        high_order = values[-1]
        reference_error = abs(high_order - expected) / abs(expected)
        cross_order_delta = abs(values[-1] - values[-2]) / abs(high_order)
        output.append(
            {
                "case_id": case["case_id"],
                "quadrature": "conditional-normal CDF, 1-D Gauss-Legendre",
                "orders": list(ORDERS),
                "integral_by_order": values,
                "frozen_polar_reference": expected,
                "relative_error_to_frozen_polar_reference": reference_error,
                "relative_delta_between_orders": cross_order_delta,
            }
        )

        if not math.isfinite(high_order) or not (0.0 < high_order <= 1.0):
            failures.append(f"{case['case_id']}: invalid integrated probability")
        if reference_error > MAX_RELATIVE_REFERENCE_ERROR:
            failures.append(
                f"{case['case_id']}: conditional integral differs from frozen "
                f"reference by {reference_error:.3e}"
            )
        if cross_order_delta > MAX_RELATIVE_CROSS_ORDER_DELTA:
            failures.append(
                f"{case['case_id']}: conditional quadrature not stable across "
                f"orders {ORDERS} ({cross_order_delta:.3e} relative delta)"
            )

    print(
        json.dumps(
            {
                "fixture": str(FIXTURE),
                "reference_method": "1-D conditional-normal CDF integral",
                "comparison_method": fixture["reference"]["integrator"],
                "results": output,
            },
            indent=2,
        )
    )

    if failures:
        print("\nCONDITIONAL REFERENCE VERIFICATION FAILED:", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
        return 1

    print("\nConditional-normal references reproduce the frozen polar integrals.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
