#!/usr/bin/env python3
"""Method figures for the bCENIC thin-objects paper (no simulation needed).

Replicates the RegularizedBarrierModel math from
multibody/contact_solvers/icf/patch_constraints_pool.cc:

  - pressure law              p(e) = E e / (1 - e)
  - elastic impulse           n_e(e) = dt A0 E* e / (1 - e)
  - near-rigid continuation:  x = 1 - e, x_nr = min(1, sqrt(k_lin / k_nr)),
        k_lin = A0 E* / (2 delta),  k_nr = (m/eps) / dt^2,
        m/eps = 4 pi^2 / (w beta^2),  w ~= 2.65 / m
    linear branch (e + x_nr >= 1):
        n~(e) = v_delta (m/eps) (e + x_nr - 1) + n_x(x_nr)
  - stiffness cap             k_nr ∝ 1 / (beta dt)^2

Figures (written to project_plan/plots/method/):
  continuation_impulse.png   n(e) with/without continuation, several beta
  continuation_slope.png     dn/de showing the C1 splice
  stiffness_cap.png          effective max stiffness vs dt (log-log), per beta
  barrier_N_n_dn.png         N, n, dn vs v_n (the solver's view)
"""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PLOTS = Path(__file__).resolve().parents[1] / "plots" / "method"

# Representative contact-pair parameters (single sphere-on-table vertex pair,
# consistent with the E1 microbenchmark scales).
DT = 0.01          # [s]
A0_E_STAR = 1e5 * 1e-4   # A0 * E*  [N]: 1 cm^2 patch at E* = 1e5 Pa... kept
                          # simple; only ratios matter for the figures.
DELTA = 1e-4       # [m] barrier layer thickness
MASS = 0.1         # [kg]
W_OF_M = 2.65 / MASS     # Delassus approximation w ~= 2.65 / m
E0 = 0.0           # extent at the start of the step
D = 10.0           # Hunt & Crossley dissipation [s/m]

BETAS = [0.25, 0.5, 1.0, 2.0, 4.0]


def model_params(beta: float, dt: float = DT):
    m_over_eps = 4 * np.pi**2 / (W_OF_M * beta**2)
    k_lin = A0_E_STAR / (2 * DELTA)
    k_nr = m_over_eps / dt**2
    x_nr = min(1.0, np.sqrt(k_lin / k_nr))
    v_delta = 2 * DELTA / dt
    return m_over_eps, x_nr, v_delta


def n_e(e, dt=DT):
    return dt * A0_E_STAR * e / (1 - e)


def n_tilde(e, beta, dt=DT):
    """Impulse with the near-rigid analytic continuation."""
    m_over_eps, x_nr, v_delta = model_params(beta, dt)
    e = np.asarray(e, dtype=float)
    n_at_xnr = dt * A0_E_STAR * (1 - x_nr) / x_nr
    linear = v_delta * m_over_eps * (e + x_nr - 1.0) + n_at_xnr
    return np.where(e + x_nr >= 1.0, linear, n_e(e, dt))


def dn_de_tilde(e, beta, dt=DT):
    m_over_eps, x_nr, v_delta = model_params(beta, dt)
    e = np.asarray(e, dtype=float)
    barrier = dt * A0_E_STAR / (1 - e) ** 2
    linear = np.full_like(e, v_delta * m_over_eps)
    return np.where(e + x_nr >= 1.0, linear, barrier)


def fig_continuation():
    e = np.linspace(0.0, 0.999, 4000)
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    ax.semilogy(e, np.maximum(n_e(e), 1e-12), "k--", lw=2,
                label="pure barrier $n_e$")
    for beta in BETAS:
        _, x_nr, _ = model_params(beta)
        ax.semilogy(e, np.maximum(n_tilde(e, beta), 1e-12),
                    label=fr"$\beta={beta}$  ($x_{{nr}}={x_nr:.3g}$)")
        ax.axvline(1 - x_nr, color="gray", alpha=0.25, lw=0.8)
    ax.set_xlabel("extent $e$")
    ax.set_ylabel("normal impulse $n$ [N·s]")
    ax.set_title("Barrier impulse and near-rigid analytic continuation")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(PLOTS / "continuation_impulse.png", dpi=200)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    ax.semilogy(e, dn_de_tilde(e, 1e9), "k--", lw=2,
                label="pure barrier $dn/de$")
    for beta in BETAS:
        ax.semilogy(e, dn_de_tilde(e, beta), label=fr"$\beta={beta}$")
    ax.set_xlabel("extent $e$")
    ax.set_ylabel("stiffness $dn/de$")
    ax.set_title("C$^1$ splice: slope is continuous and capped per $\\beta$")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(PLOTS / "continuation_slope.png", dpi=200)
    plt.close(fig)


def fig_stiffness_cap():
    dts = np.logspace(-5, -1, 100)
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for beta in BETAS:
        m_over_eps = 4 * np.pi**2 / (W_OF_M * beta**2)
        k_cap = m_over_eps / dts**2
        ax.loglog(dts, k_cap, label=fr"$\beta={beta}$")
    k_lin = A0_E_STAR / (2 * DELTA)
    ax.axhline(k_lin, color="k", ls="--", lw=1.5,
               label="physical barrier stiffness $k_{lin}$")
    ax.set_xlabel("time step $dt$ [s]")
    ax.set_ylabel("near-rigid stiffness cap $k_{nr}$")
    ax.set_title("Stiffness cap $\\propto 1/(\\beta\\, dt)^2$; "
                 "barrier recovered when $k_{nr} \\geq k_{lin}$")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(PLOTS / "stiffness_cap.png", dpi=200)
    plt.close(fig)


def fig_N_n_dn():
    """The solver's view: cost antiderivative N, impulse n, slope dn vs the
    normal velocity v (positive = separating), for beta = 1."""
    beta = 1.0
    m_over_eps, x_nr, v_delta = model_params(beta)
    d = D
    e0 = E0

    def e_of_v(v):
        return e0 - v / v_delta

    v_hat = min(v_delta * e0, 1.0 / (d + 1e-20))
    v = np.linspace(-3 * v_delta, v_hat * 0.999 if v_hat > 0 else v_delta,
                    4000)
    e = e_of_v(v)
    n = n_tilde(e, beta) * (1 - d * v)
    dn = (-dn_de_tilde(e, beta) / v_delta) * (1 - d * v) - d * n_tilde(e, beta)
    # N by numeric integration of -n? The solver minimizes -N with dN/dv = n.
    N = np.concatenate([[0.0], np.cumsum(0.5 * (n[1:] + n[:-1]) * np.diff(v))])

    fig, axes = plt.subplots(3, 1, figsize=(6.4, 7.5), sharex=True)
    axes[0].plot(v, N)
    axes[0].set_ylabel("$N(v)$")
    axes[0].set_title(fr"Barrier terms vs normal velocity ($\beta={beta}$, "
                      fr"$e_0={e0}$, $d={d}$)")
    axes[1].plot(v, n)
    axes[1].set_ylabel("$n(v)$ [N·s]")
    axes[2].plot(v, dn)
    axes[2].set_ylabel("$dn/dv$")
    axes[2].set_xlabel("normal velocity $v_n$ [m/s]")
    for ax in axes:
        ax.axvline(0, color="gray", lw=0.6, alpha=0.5)
    fig.tight_layout()
    fig.savefig(PLOTS / "barrier_N_n_dn.png", dpi=200)
    plt.close(fig)


def main():
    PLOTS.mkdir(parents=True, exist_ok=True)
    fig_continuation()
    fig_stiffness_cap()
    fig_N_n_dn()
    print(f"Method figures written to {PLOTS}")


if __name__ == "__main__":
    main()
