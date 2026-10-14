#!/usr/bin/env python3
"""Recompute the frozen correlated Gaussian-disk integral references.

Requires NumPy. This calculation is intentionally separate from Mycelix Rust
production code and uses tensor-product Gauss-Legendre quadrature in polar
coordinates. It is a numerical reference for the integral, not a flight model.
"""

from __future__ import annotations

import json
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
MAX_REFERENCE_RELATIVE_ERROR = 1e-12
MAX_CROSS_ORDER_ABSOLUTE_DELTA = 5e-19


def integrate_disk(case: dict[str, Any], radial_order: int) -> float:
    """Integrate N(q; mu, covariance) over the radius-R disk in polar form."""
    angular_order = 2 * radial_order + 24
    radius = float(case["hard_body_radius_km"])
    mean = np.asarray(case["miss_encounter_plane_km"], dtype=np.float64)
    covariance = np.asarray(
        case["covariance_encounter_plane_km2"], dtype=np.float64
    )

    if mean.shape != (2,) or covariance.shape != (2, 2):
        raise ValueError(f"{case['case_id']}: invalid vector/matrix shape")
    if not np.isfinite(mean).all() or not np.isfinite(covariance).all():
        raise ValueError(f"{case['case_id']}: non-finite input")
    if not np.isfinite(radius) or radius <= 0.0:
        raise ValueError(f"{case['case_id']}: radius must be finite and positive")
    if not np.allclose(covariance, covariance.T, rtol=1e-12, atol=0.0):
        raise ValueError(f"{case['case_id']}: covariance is not symmetric")

    sign, log_determinant = np.linalg.slogdet(covariance)
    if sign <= 0 or not np.isfinite(log_determinant):
        raise ValueError(f"{case['case_id']}: covariance must be positive definite")
    inverse = np.linalg.inv(covariance)

    r_nodes, r_weights = leggauss(radial_order)
    radii = (r_nodes + 1.0) * radius / 2.0
    radial_weights = r_weights * radius / 2.0

    theta_nodes, theta_weights = leggauss(angular_order)
    thetas = (theta_nodes + 1.0) * np.pi
    angular_weights = theta_weights * np.pi

    points = np.stack(
        (
            np.outer(radii, np.cos(thetas)),
            np.outer(radii, np.sin(thetas)),
        ),
        axis=-1,
    )
    delta = points - mean[None, None, :]
    mahalanobis_squared = np.einsum(
        "...i,ij,...j->...", delta, inverse, delta, optimize=True
    )
    density = np.exp(-0.5 * mahalanobis_squared) / (
        2.0 * np.pi * np.exp(0.5 * log_determinant)
    )

    # dA = r dr dtheta. Transform the Gauss-Legendre weights to each interval.
    return float(
        np.sum(
            density
            * radii[:, None]
            * radial_weights[:, None]
            * angular_weights[None, :]
        )
    )


def main() -> int:
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    orders = fixture["reference"]["radial_orders"]
    if not orders or orders != sorted(set(orders)):
        raise ValueError("radial_orders must be unique and strictly increasing")
    if fixture["reference"]["angular_nodes_rule"] != (
        "angular_nodes = 2 * radial_order + 24"
    ):
        raise ValueError("unrecognized angular quadrature rule")

    failures: list[str] = []
    output: list[dict[str, Any]] = []
    largest_order_delta = 0.0

    for case in [*fixture["cases"], fixture["analytic_control"]]:
        values = [integrate_disk(case, int(order)) for order in orders]
        expected = float(case["expected_pc_integral_reference"])
        actual = values[-1]
        relative_error = abs(actual - expected) / abs(expected)
        order_delta = max(abs(value - actual) for value in values[:-1])
        largest_order_delta = max(largest_order_delta, order_delta)

        record = {
            "case_id": case["case_id"],
            "radial_orders": orders,
            "integral_by_order": values,
            "frozen_reference": expected,
            "relative_error_to_frozen_reference": relative_error,
            "max_absolute_delta_from_highest_order": order_delta,
        }
        output.append(record)

        if relative_error > MAX_REFERENCE_RELATIVE_ERROR:
            failures.append(
                f"{case['case_id']}: reference mismatch {relative_error:.3e}"
            )
        if order_delta > MAX_CROSS_ORDER_ABSOLUTE_DELTA:
            failures.append(
                f"{case['case_id']}: quadrature did not converge "
                f"({order_delta:.3e} absolute delta)"
            )

    control = fixture["analytic_control"]
    control_mean = np.asarray(control["miss_encounter_plane_km"], dtype=np.float64)
    control_covariance = np.asarray(
        control["covariance_encounter_plane_km2"], dtype=np.float64
    )
    control_radius = float(control["hard_body_radius_km"])
    if control_mean.shape != (2,) or control_covariance.shape != (2, 2):
        failures.append("analytic control: invalid vector/matrix shape")
        closed_form_error = float("inf")
    elif (
        not np.array_equal(control_mean, np.zeros(2, dtype=np.float64))
        or not np.allclose(
            control_covariance,
            np.eye(2, dtype=np.float64) * control_covariance[0, 0],
            rtol=1e-14,
            atol=1e-15,
        )
        or control_covariance[0, 0] <= 0.0
    ):
        failures.append("analytic control must be centered with isotropic positive covariance")
        closed_form_error = float("inf")
    else:
        closed_form_pc = 1.0 - float(
            np.exp(-control_radius**2 / (2.0 * control_covariance[0, 0]))
        )
        closed_form_error = abs(
            float(control["expected_pc_integral_reference"]) - closed_form_pc
        ) / abs(closed_form_pc)
        control_record = next(
            (record for record in output if record["case_id"] == control["case_id"]),
            None,
        )
        if control_record is not None:
            control_record["closed_form_pc"] = closed_form_pc
            control_record["relative_error_to_closed_form"] = closed_form_error
        if closed_form_error > MAX_REFERENCE_RELATIVE_ERROR:
            failures.append(
                "analytic control: frozen integral reference differs from "
                f"closed form by {closed_form_error:.3e}"
            )

    print(json.dumps(
        {
            "fixture": str(FIXTURE),
            "integrator": fixture["reference"]["integrator"],
            "largest_cross_order_absolute_delta": largest_order_delta,
            "results": output,
        },
        indent=2,
    ))

    if failures:
        print("\nREFERENCE VERIFICATION FAILED:", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
        return 1

    print("\nAll frozen integral references reproduced within the verifier tolerance.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
