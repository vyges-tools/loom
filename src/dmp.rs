// SPDX-License-Identifier: Apache-2.0
//! Dartu/Menezes/Pileggi effective capacitance — a transcription of OpenSTA's `DmpPi`.
//!
//! This is the driver half of OpenSTA's DEFAULT arc delay calculator, `dmp_ceff_elmore`
//! (`Sta.cc:426`). Given the driver's Pi model it solves for an **effective capacitance**,
//! looks the gate tables up at that Ceff, and derives the driver's output slew from the
//! resulting **waveform** rather than from the slew table.
//!
//! ⛔ Two things here are easy to get wrong and were, before this module existed:
//!
//! 1. **Ceff is a three-equation Newton solve**, not a fixed point. The unknowns are
//!    `(t0, dt, ceff)`; the equations are a charge match between the Pi and the effective
//!    capacitance, plus the requirement that the waveform crosses **Vth** and **Vl** at the
//!    times the gate tables say (`DmpPi::evalDmpEqns`). A fixed point on the slew table
//!    returns a systematically small Ceff — measured 0.3640 pF against the reference's
//!    0.6137 on a fanout-298 sky130 net, a 41 % error that propagates into every sink.
//! 2. **The driver slew is NOT the table slew at Ceff.** `DmpPi::gateDelaySlew` takes the
//!    DELAY from the table and then overwrites the slew with `findDriverDelaySlew`'s
//!    `vo_slew` — `//slew = table_slew;` sits commented out in the reference. On that same
//!    net `report_dcalc` prints `Slew = 0.6129` from the table and
//!    `Driver waveform slew = 1.1048` from the waveform, and the timing path uses the
//!    latter: a factor of 1.80.
//!
//! **Units.** (ns, pF, kΩ) throughout, so `rd·C` and `rpi·C` come out in ns without a scale
//! factor. The reference works in SI internally and converts only for display.

/// The reference's own fast exponential (`DmpCeff.cc`, `static double exp2`), transcribed
/// rather than replaced by `f64::exp`.
///
/// ⛔ It is an APPROXIMATION on purpose — the comment there says it saves about 2.5 % of
/// run time — so `exp` would be *more accurate* than the reference, which is a divergence
/// and not an improvement. It also hard-zeros below -12, which `exp` does not.
fn exp2(x: f64) -> f64 {
    if x < -12.0 {
        0.0
    } else {
        // 12 squarings, because 2^12 = 4096 and the seed is `1 + x/4096`. ⚠️ Getting the
        // COUNT wrong is silent: eight squarings still returns a smooth positive function,
        // it is just `(1 + x/4096)^256`, which is nowhere near `exp` — `exp2(-5)` comes out
        // 0.73 instead of 0.0067. The unit test below exists for exactly that slip.
        let mut y = 1.0 + x / 4096.0;
        for _ in 0..12 {
            y *= y;
        }
        y
    }
}

/// Why a DMP solve gave up. The reference throws `DmpError` with these same reasons and
/// reports them under the `dcalc_error` debug group; each one has a defined fallback.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmpError {
    CeffNegative,
    CeffOverTotal,
    SlewZero,
    DtNonPositive,
    NewtonMaxIter,
    RootNotFound,
}

/// Library thresholds, as fractions of VDD.
#[derive(Debug, Clone, Copy)]
pub struct Vth {
    pub vth: f64,
    pub vl: f64,
    pub vh: f64,
    pub slew_derate: f64,
}

/// What the solve produced.
#[derive(Debug, Clone, Copy)]
pub struct DmpResult {
    pub ceff: f64,
    /// Gate delay, from the tables at `ceff`.
    pub delay: f64,
    /// Driver slew. The WAVEFORM slew when the driver solve succeeded, else the table slew.
    pub slew: f64,
    /// `false` when the driver solve failed and a fallback produced the numbers. The
    /// reference keeps the same flag and its sink model reads it: with `driver_valid_`
    /// false, `DmpAlg::loadDelaySlew` degrades no slew at all.
    pub driver_valid: bool,
    /// The state the per-sink solve needs. `None` when there is no waveform to hand on.
    pub waveform: Option<DriverWaveform>,
    /// Why the solve gave up, when it did. The reference reports the same reasons under
    /// its `dcalc_error` debug group; carrying it means a fallback is never silent.
    pub fail: Option<DmpError>,
}

const DRIVER_PARAM_TOL: f64 = 0.01; // `driver_param_tol`
const VTH_TIME_TOL: f64 = 0.01; // `vth_time_tol`
const FIND_ROOT_MAX_ITER: usize = 20;
const NEWTON_MAX_ITER: usize = 100;

/// One driver's DMP state: the Pi, the gate's output resistance, and the pole/residue
/// decomposition `DmpPi::init` derives from them.
pub struct DmpPi<'a> {
    rd: f64,
    c2: f64,
    rpi: f64,
    c1: f64,
    t: Vth,
    // poles/residues, verbatim from `DmpPi::init` (`z1` is only needed to form `k2`)
    k0: f64,
    p1: f64,
    p2: f64,
    k1: f64,
    k2: f64,
    k3: f64,
    k4: f64,
    a: f64,
    b: f64,
    d: f64,
    // solved
    t0: f64,
    dt: f64,
    /// `(ceff) -> (table delay, table slew)` — the gate model lookup.
    gate: &'a dyn Fn(f64) -> (f64, f64),
}

impl<'a> DmpPi<'a> {
    pub fn new(
        rd: f64,
        c2: f64,
        rpi: f64,
        c1: f64,
        t: Vth,
        gate: &'a dyn Fn(f64) -> (f64, f64),
    ) -> Self {
        // `DmpPi::init` — poles and zeros of the Pi driven through Rd.
        let z1 = 1.0 / (rpi * c1);
        let k0 = 1.0 / (rd * c2);
        let a_ = rpi * rd * c1 * c2;
        let b_ = rd * (c1 + c2) + rpi * c1;
        let sqrt_ = (b_ * b_ - 4.0 * a_).sqrt();
        let p1 = (b_ + sqrt_) / (2.0 * a_);
        let p2 = (b_ - sqrt_) / (2.0 * a_);
        let p1p2 = p1 * p2;
        let k2 = z1 / p1p2;
        let k1 = (1.0 - k2 * (p1 + p2)) / p1p2;
        let k4 = (k1 * p1 + k2) / (p2 - p1);
        let k3 = -k1 - k4;
        let z_ = (c1 + c2) / (rpi * c1 * c2);
        DmpPi {
            rd,
            c2,
            rpi,
            c1,
            t,
            k0,
            p1,
            p2,
            k1,
            k2,
            k3,
            k4,
            a: z_ / p1p2,
            b: (z_ - p1) / (p1 * (p1 - p2)),
            d: (z_ - p2) / (p2 * (p2 - p1)),
            t0: 0.0,
            dt: 0.0,
            gate,
        }
    }

    /// `DmpAlg::gateDelays` — table delay at `ceff`, plus the MEASURED slew (the table slew
    /// scaled by `slew_derate`) and the Vl crossing time implied by it.
    fn gate_delays(&self, ceff: f64) -> (f64, f64, f64) {
        let (t_vth, table_slew) = (self.gate)(ceff);
        let slew = table_slew * self.t.slew_derate;
        let t_vl = t_vth - slew * (self.t.vth - self.t.vl) / (self.t.vh - self.t.vl);
        (t_vth, t_vl, slew)
    }

    // ---- the driver waveform, `DmpAlg::y` / `y0` / `dy` ----------------------------
    fn y0(&self, t: f64, cl: f64) -> f64 {
        t - self.rd * cl * (1.0 - exp2(-t / (self.rd * cl)))
    }
    fn y0dt(&self, t: f64, cl: f64) -> f64 {
        1.0 - exp2(-t / (self.rd * cl))
    }
    fn y0dcl(&self, t: f64, cl: f64) -> f64 {
        self.rd * ((1.0 + t / (self.rd * cl)) * exp2(-t / (self.rd * cl)) - 1.0)
    }
    fn y(&self, t: f64, t0: f64, dt: f64, cl: f64) -> f64 {
        let t1 = t - t0;
        if t1 <= 0.0 {
            0.0
        } else if t1 <= dt {
            self.y0(t1, cl) / dt
        } else {
            (self.y0(t1, cl) - self.y0(t1 - dt, cl)) / dt
        }
    }
    /// Partials of `y` w.r.t. (t0, dt, cl) — the Jacobian rows for the two crossing
    /// equations.
    fn dy(&self, t: f64, t0: f64, dt: f64, cl: f64) -> (f64, f64, f64) {
        let t1 = t - t0;
        if t1 <= 0.0 {
            (0.0, 0.0, 0.0)
        } else if t1 <= dt {
            (
                -self.y0dt(t1, cl) / dt,
                -self.y0(t1, cl) / (dt * dt),
                self.y0dcl(t1, cl) / dt,
            )
        } else {
            (
                -(self.y0dt(t1, cl) - self.y0dt(t1 - dt, cl)) / dt,
                -(self.y0(t1, cl) + self.y0(t1 - dt, cl)) / (dt * dt)
                    + self.y0dt(t1 - dt, cl) / dt,
                (self.y0dcl(t1, cl) - self.y0dcl(t1 - dt, cl)) / dt,
            )
        }
    }

    /// `DmpPi::ipiIceff` — eqn 13/14: the charge the Pi draws minus the charge `ceff`
    /// draws, over the same interval. Driving this to zero is what makes `ceff` effective.
    fn ipi_iceff(&self, dt: f64, ceff_time: f64, ceff: f64) -> f64 {
        let exp_p1 = exp2(-self.p1 * ceff_time);
        let exp_p2 = exp2(-self.p2 * ceff_time);
        let exp_rd = exp2(-ceff_time / (self.rd * ceff));
        let ipi = (self.a * ceff_time
            + (self.b / self.p1) * (1.0 - exp_p1)
            + (self.d / self.p2) * (1.0 - exp_p2))
            / (self.rd * ceff_time * dt);
        let iceff = (self.rd * ceff * ceff_time
            - (self.rd * ceff) * (self.rd * ceff) * (1.0 - exp_rd))
            / (self.rd * ceff_time * dt);
        ipi - iceff
    }

    /// `DmpPi::evalDmpEqns` — residuals and Jacobian at `x = [t0, dt, ceff]`.
    /// Order matches the reference: row 0 = charge match, row 1 = Vth crossing (`y50`),
    /// row 2 = Vl crossing (`y20`).
    fn eval(&self, x: &[f64; 3]) -> Result<([f64; 3], [[f64; 3]; 3]), DmpError> {
        let (t0, dt, ceff) = (x[0], x[1], x[2]);
        if ceff < 0.0 {
            return Err(DmpError::CeffNegative);
        }
        if ceff > self.c1 + self.c2 {
            return Err(DmpError::CeffOverTotal);
        }
        let (t_vth, t_vl, slew) = self.gate_delays(ceff);
        if slew == 0.0 {
            return Err(DmpError::SlewZero);
        }
        // the charge-matching interval, capped at 1.4*dt as the reference caps it
        let mut ceff_time = slew / (self.t.vh - self.t.vl);
        if ceff_time > 1.4 * dt {
            ceff_time = 1.4 * dt;
        }
        if dt <= 0.0 {
            return Err(DmpError::DtNonPositive);
        }
        let exp_p1_dt = exp2(-self.p1 * dt);
        let exp_p2_dt = exp2(-self.p2 * dt);
        let exp_dt_rd_ceff = exp2(-dt / (self.rd * ceff));

        let mut f = [0.0f64; 3];
        let mut j = [[0.0f64; 3]; 3];
        f[0] = self.ipi_iceff(dt, ceff_time, ceff);
        f[1] = self.y(t_vth, t0, dt, ceff) - self.t.vth;
        f[2] = self.y(t_vl, t0, dt, ceff) - self.t.vl;

        j[0][0] = 0.0;
        j[0][1] = (-self.a * dt + self.b * dt * exp_p1_dt
            - (2.0 * self.b / self.p1) * (1.0 - exp_p1_dt)
            + self.d * dt * exp_p2_dt
            - (2.0 * self.d / self.p2) * (1.0 - exp_p2_dt)
            + self.rd
                * ceff
                * (dt + dt * exp_dt_rd_ceff
                    - 2.0 * self.rd * ceff * (1.0 - exp_dt_rd_ceff)))
            / (self.rd * dt * dt * dt);
        j[0][2] = (2.0 * self.rd * ceff
            - dt
            - (2.0 * self.rd * ceff + dt) * exp2(-dt / (self.rd * ceff)))
            / (dt * dt);
        let (a0, a1, a2) = self.dy(t_vth, t0, dt, ceff);
        j[1] = [a0, a1, a2];
        let (b0, b1, b2) = self.dy(t_vl, t0, dt, ceff);
        j[2] = [b0, b1, b2];
        Ok((f, j))
    }

    /// `DmpAlg::findDriverParams` — seed `(t0, dt)` from the tables at `ceff`, then
    /// Newton-Raphson over all three unknowns.
    fn find_driver_params(&mut self, ceff_seed: f64) -> Result<f64, DmpError> {
        let (t_vth, _t_vl, slew) = self.gate_delays(ceff_seed);
        let dt = slew / (self.t.vh - self.t.vl);
        let t0 = t_vth + (1.0 - self.t.vth).ln() * self.rd * ceff_seed - self.t.vth * dt;
        let mut x = [t0, dt, ceff_seed];
        for _ in 0..NEWTON_MAX_ITER {
            let (f, mut j) = self.eval(&x)?;
            let p = solve3(&mut j, [-f[0], -f[1], -f[2]])?;
            let mut done = true;
            for i in 0..3 {
                if p[i].abs() > x[i].abs() * DRIVER_PARAM_TOL {
                    done = false;
                }
                x[i] += p[i];
            }
            if done {
                self.t0 = x[0];
                self.dt = x[1];
                return Ok(x[2]);
            }
        }
        Err(DmpError::NewtonMaxIter)
    }

    // ---- the driver-output waveform, `DmpPi::V0` / `DmpAlg::Vo` --------------------
    fn v0(&self, t: f64) -> (f64, f64) {
        let e1 = exp2(-self.p1 * t);
        let e2 = exp2(-self.p2 * t);
        (
            self.k0 * (self.k1 + self.k2 * t + self.k3 * e1 + self.k4 * e2),
            self.k0 * (self.k2 - self.k3 * self.p1 * e1 - self.k4 * self.p2 * e2),
        )
    }
    fn vo(&self, t: f64) -> (f64, f64) {
        let t1 = t - self.t0;
        if t1 <= 0.0 {
            (0.0, 0.0)
        } else if t1 <= self.dt {
            let (v, dv) = self.v0(t1);
            (v / self.dt, dv / self.dt)
        } else {
            let (v, dv) = self.v0(t1);
            let (v2, dv2) = self.v0(t1 - self.dt);
            ((v - v2) / self.dt, (dv - dv2) / self.dt)
        }
    }
    /// `DmpAlg::findVoCrossing` — safeguarded root find for `Vo(t) = v`.
    fn find_vo_crossing(&self, v: f64, lo: f64, hi: f64) -> Result<f64, DmpError> {
        let (mut lo, mut hi) = (lo, hi);
        let mut t = 0.5 * (lo + hi);
        for _ in 0..FIND_ROOT_MAX_ITER {
            let (vo, dvo) = self.vo(t);
            let err = vo - v;
            if err.abs() < VTH_TIME_TOL * v.max(1e-12) {
                return Ok(t);
            }
            if err > 0.0 {
                hi = t;
            } else {
                lo = t;
            }
            // Newton step when it stays inside the bracket, bisection otherwise —
            // the shape of the reference's `findRoot`.
            let next = if dvo.abs() > 0.0 { t - err / dvo } else { f64::NAN };
            t = if next.is_finite() && next > lo && next < hi {
                next
            } else {
                0.5 * (lo + hi)
            };
        }
        Err(DmpError::RootNotFound)
    }
    /// `DmpAlg::findDriverDelaySlew` — the waveform's Vth crossing and its Vl→Vh slew.
    fn find_driver_delay_slew(&self) -> Result<(f64, f64), DmpError> {
        // `DmpPi::voCrossingUpperBound`
        let t_upper = self.t0 + self.dt + (self.c1 + self.c2) * (self.rd + self.rpi) * 2.0;
        let delay = self.find_vo_crossing(self.t.vth, self.t0, t_upper)?;
        let tl = self.find_vo_crossing(self.t.vl, self.t0, delay)?;
        let th = self.find_vo_crossing(self.t.vh, delay, t_upper)?;
        // convert the MEASURED slew back to a table slew
        Ok((delay, (th - tl) / self.t.slew_derate))
    }

    /// Hand the solved state to the per-sink model.
    fn waveform(&self, vo_delay: f64, drvr_slew: f64, driver_valid: bool) -> DriverWaveform {
        DriverWaveform {
            t0: self.t0,
            dt: self.dt,
            vo_delay,
            drvr_slew,
            driver_valid,
            k0: self.k0,
            k1: self.k1,
            k2: self.k2,
            k3: self.k3,
            k4: self.k4,
            p1: self.p1,
            p2: self.p2,
            rd: self.rd,
            rpi: self.rpi,
            c1: self.c1,
            c2: self.c2,
            t: self.t,
        }
    }

    /// `DmpPi::gateDelaySlew`, fallbacks included.
    pub fn solve(&mut self) -> DmpResult {
        // `findDriverParamsPi`: try the whole load, then the near capacitance alone.
        let ceff = self
            .find_driver_params(self.c2 + self.c1)
            .or_else(|_| self.find_driver_params(self.c2));
        match ceff {
            Ok(ceff) => {
                let (delay, table_slew) = (self.gate)(ceff);
                match self.find_driver_delay_slew() {
                    // ⛔ the slew is the WAVEFORM's, not the table's
                    Ok((vo_delay, vo_slew)) => DmpResult {
                        ceff,
                        delay,
                        slew: vo_slew,
                        driver_valid: true,
                        waveform: Some(self.waveform(vo_delay, vo_slew, true)),
                        fail: None,
                    },
                    // "Fall back to table slew" — the delay still stands
                    Err(e) => DmpResult {
                        ceff,
                        delay,
                        slew: table_slew,
                        driver_valid: false,
                        waveform: Some(self.waveform(0.0, table_slew, false)),
                        fail: Some(e),
                    },
                }
            }
            // "Driver calculation failed - use Ceff=c1+c2"
            Err(e) => {
                let ceff = self.c1 + self.c2;
                let (delay, slew) = (self.gate)(ceff);
                DmpResult {
                    ceff,
                    delay,
                    slew,
                    driver_valid: false,
                    waveform: Some(self.waveform(0.0, slew, false)),
                    fail: Some(e),
                }
            }
        }
    }
}

/// 3x3 solve with partial pivoting, standing in for the reference's `luDecomp`/`luSolve`
/// (Crout with implicit pivoting). ⚠️ Same answer to rounding on a well-conditioned 3x3;
/// the pivoting rule differs, which is a deliberate and documented simplification.
fn solve3(a: &mut [[f64; 3]; 3], mut b: [f64; 3]) -> Result<[f64; 3], DmpError> {
    for col in 0..3 {
        let piv = (col..3).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()));
        let piv = piv.ok_or(DmpError::NewtonMaxIter)?;
        if a[piv][col].abs() < 1e-300 {
            return Err(DmpError::NewtonMaxIter);
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for row in (col + 1)..3 {
            let f = a[row][col] / a[col][col];
            let pivot_row = a[col];
            for (k, v) in a[row].iter_mut().enumerate().skip(col) {
                *v -= f * pivot_row[k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0f64; 3];
    for row in (0..3).rev() {
        let mut s = b[row];
        for k in (row + 1)..3 {
            s -= a[row][k] * x[k];
        }
        x[row] = s / a[row][row];
    }
    if x.iter().any(|v| !v.is_finite()) {
        return Err(DmpError::NewtonMaxIter);
    }
    Ok(x)
}

/// Everything the PER-SINK solve needs from the driver solve that ran before it.
///
/// 🔑 This is a call-sequence coupling, not a convenience: `DmpAlg::loadDelaySlew` reads
/// `t0_`, `dt_`, `vo_delay_`, `drvr_slew_` and `driver_valid_`, all set by
/// `gateDelaySlew` for the SAME driver. Upstream keeps them as members and relies on the
/// two being called in order within one arc evaluation; we hand them over explicitly so
/// the order cannot be got wrong silently.
#[derive(Debug, Clone, Copy)]
pub struct DriverWaveform {
    t0: f64,
    dt: f64,
    vo_delay: f64,
    drvr_slew: f64,
    driver_valid: bool,
    // the Pi's poles/residues, needed again for the sink waveform
    k0: f64,
    k1: f64,
    k2: f64,
    k3: f64,
    k4: f64,
    p1: f64,
    p2: f64,
    rd: f64,
    rpi: f64,
    c1: f64,
    c2: f64,
    t: Vth,
}

impl DriverWaveform {
    /// The capacitive case: no waveform, so every sink takes `delay = elmore` and the
    /// driver's own slew — `DmpCap::loadDelaySlew`.
    fn capacitive(drvr_slew: f64, t: Vth) -> DriverWaveform {
        DriverWaveform {
            t0: 0.0,
            dt: 0.0,
            vo_delay: 0.0,
            drvr_slew,
            driver_valid: false,
            k0: 0.0,
            k1: 0.0,
            k2: 0.0,
            k3: 0.0,
            k4: 0.0,
            p1: 0.0,
            p2: 0.0,
            rd: 0.0,
            rpi: 0.0,
            c1: 0.0,
            c2: 0.0,
            t,
        }
    }

    /// `DmpPi::Vl0` — the waveform at a LOAD, i.e. the driver waveform with a third pole
    /// at `p3 = 1/elmore` for the wire between them.
    fn vl0(&self, t: f64, p3: f64) -> (f64, f64) {
        let d1 = self.k0 * (self.k1 - self.k2 / p3);
        let d3 = -p3 * self.k0 * self.k3 / (self.p1 - p3);
        // one-pole case: k4 = 0 and p2 = INFINITY, so this term is zero by construction
        let d4 = if self.k4 == 0.0 { 0.0 } else { -p3 * self.k0 * self.k4 / (self.p2 - p3) };
        let d5 = self.k0
            * (self.k2 / p3 - self.k1 + p3 * self.k3 / (self.p1 - p3)
                + if self.k4 == 0.0 { 0.0 } else { p3 * self.k4 / (self.p2 - p3) });
        let e1 = exp2(-self.p1 * t);
        let e2 = exp2(-self.p2 * t);
        let e3 = exp2(-p3 * t);
        (
            d1 + t + d3 * e1 + d4 * e2 + d5 * e3,
            1.0 - d3 * self.p1 * e1 - d4 * self.p2 * e2 - d5 * p3 * e3,
        )
    }

    /// `DmpAlg::Vl` — the ramp response built from `Vl0`, same shape as `Vo`.
    fn vl(&self, t: f64, p3: f64) -> (f64, f64) {
        let t1 = t - self.t0;
        if t1 <= 0.0 {
            (0.0, 0.0)
        } else if t1 <= self.dt {
            let (v, dv) = self.vl0(t1, p3);
            (v / self.dt, dv / self.dt)
        } else {
            let (v, dv) = self.vl0(t1, p3);
            let (v2, dv2) = self.vl0(t1 - self.dt, p3);
            ((v - v2) / self.dt, (dv - dv2) / self.dt)
        }
    }

    fn find_vl_crossing(&self, v: f64, lo: f64, hi: f64, p3: f64) -> Result<f64, DmpError> {
        let (mut lo, mut hi) = (lo, hi);
        let mut t = 0.5 * (lo + hi);
        for _ in 0..FIND_ROOT_MAX_ITER {
            let (vl, dvl) = self.vl(t, p3);
            let err = vl - v;
            if err.abs() < VTH_TIME_TOL * v.max(1e-12) {
                return Ok(t);
            }
            if err > 0.0 {
                hi = t;
            } else {
                lo = t;
            }
            let next = if dvl.abs() > 0.0 { t - err / dvl } else { f64::NAN };
            t = if next.is_finite() && next > lo && next < hi {
                next
            } else {
                0.5 * (lo + hi)
            };
        }
        Err(DmpError::RootNotFound)
    }

    /// `DmpAlg::loadDelaySlew` — the wire delay and the DEGRADED slew at one sink, given
    /// that sink's Elmore time constant.
    ///
    /// ⛔ The degenerate branch is not a corner case, it is most of the design: when the
    /// driver solve did not produce a waveform, or the Elmore is small against the driver
    /// slew, the wire delay IS the Elmore value and the slew is the driver's, undegraded.
    /// `DmpCap` overrides the whole method with exactly that.
    pub fn load_delay_slew(&self, elmore: f64) -> (f64, f64) {
        if !self.driver_valid || elmore == 0.0 || elmore < self.drvr_slew * 1e-3 {
            return (elmore, self.drvr_slew);
        }
        let p3 = 1.0 / elmore;
        // `vlCrossingUpperBound` = `voCrossingUpperBound` + 2*elmore
        let t_upper = self.t0
            + self.dt
            + (self.c1 + self.c2) * (self.rd + self.rpi) * 2.0
            + elmore * 2.0;
        let solved = (|| -> Result<(f64, f64), DmpError> {
            let load_delay = self.find_vl_crossing(self.t.vth, self.t0, t_upper, p3)?;
            let tl = self.find_vl_crossing(self.t.vl, self.t0, load_delay, p3)?;
            let th = self.find_vl_crossing(self.t.vh, load_delay, t_upper, p3)?;
            Ok((load_delay - self.vo_delay, (th - tl) / self.t.slew_derate))
        })();
        match solved {
            // upstream's two guards, and both fall back rather than propagate
            Ok((delay, slew)) => (
                if delay < 0.0 { elmore } else { delay },
                if slew < self.drvr_slew { self.drvr_slew } else { slew },
            ),
            Err(_) => (elmore, self.drvr_slew),
        }
    }
}

/// `DmpZeroC2` — the near capacitance is negligible, so `Ceff` is known (`= c1`) and only
/// the waveform has to be solved.
///
/// ⛔ This case is NOT rare, whatever a big block suggests: a net with a single lumped
/// capacitance behind a single resistor reduces to `c2 = 0` EXACTLY, because moment
/// matching on one R and one C gives `c1 = C`, `c2 = y1 − c1 = 0`, `rpi = R`. `fft_top`
/// never selects it and a two-element SPEF always does.
///
/// It differs from `DmpPi` in three ways worth keeping straight: `ceff` is fixed rather
/// than solved, so the Newton system is TWO equations over `(t0, dt)`; there is one pole
/// rather than two; and the gate DELAY comes from the waveform (`vo_delay_ = delay`)
/// rather than from the table.
struct DmpZeroC2<'a> {
    rd: f64,
    rpi: f64,
    c1: f64,
    t: Vth,
    k0: f64,
    k1: f64,
    k2: f64,
    k3: f64,
    p1: f64,
    t0: f64,
    dt: f64,
    gate: &'a dyn Fn(f64) -> (f64, f64),
}

impl<'a> DmpZeroC2<'a> {
    fn new(rd: f64, rpi: f64, c1: f64, t: Vth, gate: &'a dyn Fn(f64) -> (f64, f64)) -> Self {
        let z1 = 1.0 / (rpi * c1);
        let p1 = 1.0 / (c1 * (rd + rpi));
        let k0 = p1 / z1;
        let k2 = 1.0 / k0;
        let k1 = (p1 - z1) / (p1 * p1);
        DmpZeroC2 { rd, rpi, c1, t, k0, k1, k2, k3: -k1, p1, t0: 0.0, dt: 0.0, gate }
    }
    fn gate_delays(&self, ceff: f64) -> (f64, f64, f64) {
        let (t_vth, table_slew) = (self.gate)(ceff);
        let slew = table_slew * self.t.slew_derate;
        (t_vth, t_vth - slew * (self.t.vth - self.t.vl) / (self.t.vh - self.t.vl), slew)
    }
    fn y0(&self, t: f64, cl: f64) -> f64 {
        t - self.rd * cl * (1.0 - exp2(-t / (self.rd * cl)))
    }
    fn y0dt(&self, t: f64, cl: f64) -> f64 {
        1.0 - exp2(-t / (self.rd * cl))
    }
    fn y(&self, t: f64, t0: f64, dt: f64, cl: f64) -> f64 {
        let t1 = t - t0;
        if t1 <= 0.0 {
            0.0
        } else if t1 <= dt {
            self.y0(t1, cl) / dt
        } else {
            (self.y0(t1, cl) - self.y0(t1 - dt, cl)) / dt
        }
    }
    /// Partials w.r.t. (t0, dt) only — `ceff` is not an unknown here.
    fn dy(&self, t: f64, t0: f64, dt: f64, cl: f64) -> (f64, f64) {
        let t1 = t - t0;
        if t1 <= 0.0 {
            (0.0, 0.0)
        } else if t1 <= dt {
            (-self.y0dt(t1, cl) / dt, -self.y0(t1, cl) / (dt * dt))
        } else {
            (
                -(self.y0dt(t1, cl) - self.y0dt(t1 - dt, cl)) / dt,
                -(self.y0(t1, cl) + self.y0(t1 - dt, cl)) / (dt * dt)
                    + self.y0dt(t1 - dt, cl) / dt,
            )
        }
    }
    fn v0(&self, t: f64) -> (f64, f64) {
        let e1 = exp2(-self.p1 * t);
        (
            self.k0 * (self.k1 + self.k2 * t + self.k3 * e1),
            self.k0 * (self.k2 - self.k3 * self.p1 * e1),
        )
    }
    fn vo(&self, t: f64) -> (f64, f64) {
        let t1 = t - self.t0;
        if t1 <= 0.0 {
            (0.0, 0.0)
        } else if t1 <= self.dt {
            let (v, dv) = self.v0(t1);
            (v / self.dt, dv / self.dt)
        } else {
            let (v, dv) = self.v0(t1);
            let (v2, dv2) = self.v0(t1 - self.dt);
            ((v - v2) / self.dt, (dv - dv2) / self.dt)
        }
    }
    fn find_vo_crossing(&self, v: f64, lo: f64, hi: f64) -> Result<f64, DmpError> {
        let (mut lo, mut hi) = (lo, hi);
        let mut t = 0.5 * (lo + hi);
        for _ in 0..FIND_ROOT_MAX_ITER {
            let (vo, dvo) = self.vo(t);
            let err = vo - v;
            if err.abs() < VTH_TIME_TOL * v.max(1e-12) {
                return Ok(t);
            }
            if err > 0.0 {
                hi = t;
            } else {
                lo = t;
            }
            let next = if dvo.abs() > 0.0 { t - err / dvo } else { f64::NAN };
            t = if next.is_finite() && next > lo && next < hi { next } else { 0.5 * (lo + hi) };
        }
        Err(DmpError::RootNotFound)
    }
    /// `DmpOnePole::evalDmpEqns` + `DmpAlg::findDriverParams` at `nr_order == 2`.
    fn solve(&mut self) -> DmpResult {
        let (t_vth, _tvl, slew) = self.gate_delays(self.c1);
        let dt = slew / (self.t.vh - self.t.vl);
        let t0 = t_vth + (1.0 - self.t.vth).ln() * self.rd * self.c1 - self.t.vth * dt;
        let mut x = [t0, dt];
        let mut ok = false;
        for _ in 0..NEWTON_MAX_ITER {
            let (t_vth, t_vl, _s) = self.gate_delays(self.c1);
            // the reference repairs a non-positive dt in place rather than failing
            if x[1] <= 0.0 {
                x[1] = (t_vl - t_vth) / 100.0;
            }
            let f = [
                self.y(t_vth, x[0], x[1], self.c1) - self.t.vth,
                self.y(t_vl, x[0], x[1], self.c1) - self.t.vl,
            ];
            let (a0, a1) = self.dy(t_vth, x[0], x[1], self.c1);
            let (b0, b1) = self.dy(t_vl, x[0], x[1], self.c1);
            let det = a0 * b1 - a1 * b0;
            if det.abs() < 1e-300 || !det.is_finite() {
                break;
            }
            let p = [
                (-f[0] * b1 + f[1] * a1) / det,
                (-a0 * f[1] + b0 * f[0]) / det,
            ];
            let mut done = true;
            for i in 0..2 {
                if p[i].abs() > x[i].abs() * DRIVER_PARAM_TOL {
                    done = false;
                }
                x[i] += p[i];
            }
            if done {
                ok = true;
                break;
            }
        }
        self.t0 = x[0];
        self.dt = x[1];
        let (table_delay, table_slew) = (self.gate)(self.c1);
        if ok {
            // ⚠️ Unlike `DmpPi`, the DELAY comes from the waveform too — `vo_delay_ = delay`.
            let t_upper = self.t0 + self.dt + self.c1 * (self.rd + self.rpi) * 2.0;
            let r = (|| -> Result<(f64, f64), DmpError> {
                let d = self.find_vo_crossing(self.t.vth, self.t0, t_upper)?;
                let tl = self.find_vo_crossing(self.t.vl, self.t0, d)?;
                let th = self.find_vo_crossing(self.t.vh, d, t_upper)?;
                Ok((d, (th - tl) / self.t.slew_derate))
            })();
            if let Ok((delay, slew)) = r {
                return DmpResult {
                    ceff: self.c1,
                    delay,
                    slew,
                    driver_valid: true,
                    waveform: Some(DriverWaveform {
                        t0: self.t0,
                        dt: self.dt,
                        vo_delay: delay,
                        drvr_slew: slew,
                        driver_valid: true,
                        // one pole: k4/p2 are zero, which makes `Vl0`'s D4 term vanish and
                        // leaves exactly `DmpZeroC2::Vl0`.
                        k0: self.k0,
                        k1: self.k1,
                        k2: self.k2,
                        k3: self.k3,
                        k4: 0.0,
                        p1: self.p1,
                        p2: f64::INFINITY,
                        rd: self.rd,
                        rpi: self.rpi,
                        c1: self.c1,
                        c2: 0.0,
                        t: self.t,
                    }),
                    fail: None,
                };
            }
        }
        // "Fall back to table slew."
        DmpResult {
            ceff: self.c1,
            delay: table_delay,
            slew: table_slew,
            driver_valid: false,
            waveform: Some(DriverWaveform::capacitive(table_slew, self.t)),
            fail: Some(DmpError::NewtonMaxIter),
        }
    }
}

/// Which DMP algorithm the reference picks for a driver — `DmpCeffDelayCalc::setCeffAlgorithm`.
///
/// ⚠️ **One threshold is unit-bearing.** The reference works in SI, so `rd < 1e-2` means
/// "under a hundredth of an OHM", i.e. essentially no output resistance. In this module's
/// kΩ that is `1e-5`. Every other test is a ratio and so unit-free.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Alg {
    /// The load is effectively a lump: `ceff = c1 + c2`, and no waveform solve.
    Cap,
    /// Negligible near capacitance — its own reduced solve upstream.
    ZeroC2,
    /// "The full monty" — the three-equation Newton solve.
    Pi,
}

pub fn select_alg(rd: f64, c2: f64, rpi: f64, c1: f64) -> Alg {
    if rd < 1e-5            // `rd < 1e-2` ohms; table is constant, load looks capacitive
        || rpi < rd * 1e-3  // Rpi small against Rd
        || c1 == 0.0
        || c1 < c2 * 1e-3   // c1/Rpi can be ignored
        || rpi == 0.0
    {
        Alg::Cap
    } else if c2 < c1 * 1e-3 {
        Alg::ZeroC2
    } else {
        Alg::Pi
    }
}

/// Solve a driver the way the reference would, choosing the algorithm first.
///
/// ⬜ `ZeroC2` is NOT transcribed. Upstream gives it its own two-unknown solve; here it
/// takes its `init` value `ceff = c1` with table delay and slew, and reports
/// `driver_valid: false` so nothing downstream mistakes it for a waveform result. It is
/// selected only when the near capacitance is under a thousandth of the far one — and
/// routing it into `DmpPi` instead would be worse than declining: `k0 = 1/(rd·c2)` blows
/// up as `c2` goes to zero, which is the reason the reference separates the case at all.
pub fn solve_driver(
    rd: f64,
    c2: f64,
    rpi: f64,
    c1: f64,
    t: Vth,
    gate: &dyn Fn(f64) -> (f64, f64),
) -> DmpResult {
    match select_alg(rd, c2, rpi, c1) {
        Alg::Pi => DmpPi::new(rd, c2, rpi, c1, t, gate).solve(),
        // `DmpCap::init` sets `ceff_ = c1 + c2`; `gateDelaySlew` then takes BOTH the delay
        // and the slew from the tables — there is no waveform for a capacitive load.
        Alg::Cap => {
            let ceff = c1 + c2;
            let (delay, slew) = gate(ceff);
            // `DmpCap::loadDelaySlew` is `delay = elmore; slew = drvr_slew;` — exactly what
            // a `DriverWaveform` with `driver_valid: false` produces, so the sink model
            // needs no separate branch.
            DmpResult {
                ceff,
                delay,
                slew,
                driver_valid: false,
                waveform: Some(DriverWaveform::capacitive(slew, t)),
                fail: None,
            }
        }
        Alg::ZeroC2 => DmpZeroC2::new(rd, rpi, c1, t, gate).solve(),
    }
}

/// `gateModelRd` — the driver's output resistance, from two table lookups a hair apart.
/// ⚠️ The reference perturbs by `1e-15` FARADS; in our pF units that is `1e-3`.
pub fn gate_model_rd(vth: f64, gate: &dyn Fn(f64) -> (f64, f64), c1: f64, c2: f64) -> f64 {
    let cap1 = c1 + c2;
    let cap2 = cap1 + 1e-3;
    let (d1, _) = gate(cap1);
    let (d2, _) = gate(cap2);
    -(vth.ln()) * (d1 - d2).abs() / (cap2 - cap1)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// sky130-ish thresholds.
    fn th() -> Vth {
        Vth { vth: 0.5, vl: 0.2, vh: 0.8, slew_derate: 1.0 }
    }

    /// A stand-in gate table: delay and slew rise linearly with load. Monotonic and
    /// smooth, so any disagreement is the solver's and not the table's.
    fn linear_gate(ceff: f64) -> (f64, f64) {
        (0.10 + 0.35 * ceff, 0.05 + 0.50 * ceff)
    }

    /// `fft_top`'s `input55` Pi, as `report_dcalc` reports it, in (ns, pF, kΩ).
    fn input55() -> (f64, f64, f64) {
        (0.1879, 0.6019, 1.0222) // (C2 near, Rpi, C1 far)
    }

    #[test]
    fn ceff_lands_between_the_near_cap_and_the_total() {
        // The physical bracket, and the reference asserts both ends itself:
        // `evalDmpEqns` throws on `ceff < 0` and on `ceff > c1 + c2`.
        let (c2, rpi, c1) = input55();
        let rd = gate_model_rd(th().vth, &linear_gate, c1, c2);
        let r = DmpPi::new(rd, c2, rpi, c1, th(), &linear_gate).solve();
        assert!(
            r.ceff > 0.0 && r.ceff <= c1 + c2 + 1e-9,
            "Ceff {} outside (0, {}]",
            r.ceff,
            c1 + c2
        );
    }

    /// ⛔ The whole reason this module exists. A fixed point on the slew table returns a
    /// systematically SMALL Ceff; on `fft_top`'s `net55` it gave 0.3640 pF where the
    /// reference solves 0.6137 — 41 % low, and it propagated into every sink.
    #[test]
    fn ceff_is_not_collapsed_toward_the_near_cap() {
        let (c2, rpi, c1) = input55();
        let rd = gate_model_rd(th().vth, &linear_gate, c1, c2);
        let r = DmpPi::new(rd, c2, rpi, c1, th(), &linear_gate).solve();
        assert!(
            r.ceff > c2 * 1.5,
            "Ceff {} has collapsed toward the near cap {c2}; the far capacitance is only \
             PARTLY shielded, never fully",
            r.ceff
        );
    }

    /// ⛔ The second half: the driver slew is NOT the table slew at Ceff.
    /// `DmpPi::gateDelaySlew` overwrites it with the waveform's, and `//slew = table_slew;`
    /// is commented out in the reference. `report_dcalc` on `input55/X` prints
    /// `Slew = 0.6129` (table) and `Driver waveform slew = 1.1048` — a factor of 1.80.
    #[test]
    fn the_driver_slew_comes_from_the_waveform_not_the_table() {
        let (c2, rpi, c1) = input55();
        let rd = gate_model_rd(th().vth, &linear_gate, c1, c2);
        let mut d = DmpPi::new(rd, c2, rpi, c1, th(), &linear_gate);
        let r = d.solve();
        if r.driver_valid {
            let (_, table_slew) = linear_gate(r.ceff);
            assert!(
                (r.slew - table_slew).abs() > 1e-9,
                "driver slew {} is exactly the table slew at Ceff — the waveform solve is \
                 not being used",
                r.slew
            );
        }
    }

    /// 📌 From OpenSTA issue **#405, "DMP divergence with 0 input slew"** — a real bug in
    /// the DMP setup that reached the resizer. Our own `dmp_ceff` trace on `fft_top` shows
    /// `DMP in_slew = 0.000` on many evaluations, so this input is not hypothetical.
    /// It must not diverge, NaN, or return a Ceff outside the bracket: the reference's
    /// contract is that a failed solve FALLS BACK to `ceff = c1 + c2`.
    #[test]
    fn a_zero_input_slew_does_not_diverge_issue_405() {
        let flat = |_c: f64| (0.0, 0.0); // zero delay and zero slew at every load
        let (c2, rpi, c1) = input55();
        let r = DmpPi::new(1.0, c2, rpi, c1, th(), &flat).solve();
        assert!(r.ceff.is_finite() && r.slew.is_finite() && r.delay.is_finite());
        assert_eq!(
            r.ceff,
            c1 + c2,
            "a solve that cannot proceed must fall back to the TOTAL capacitance, as \
             `DmpPi::gateDelaySlew` does"
        );
        assert!(!r.driver_valid, "and it must say the driver result is not valid");
    }

    /// The reference's guards, each with its own `DmpError`. A solver that silently
    /// accepted these would return a number with no physical meaning.
    #[test]
    fn the_reference_guards_are_enforced() {
        let (c2, rpi, c1) = input55();
        let d = DmpPi::new(1.0, c2, rpi, c1, th(), &linear_gate);
        assert_eq!(d.eval(&[0.0, 1.0, -0.1]).unwrap_err(), DmpError::CeffNegative);
        assert_eq!(
            d.eval(&[0.0, 1.0, c1 + c2 + 1.0]).unwrap_err(),
            DmpError::CeffOverTotal
        );
        assert_eq!(d.eval(&[0.0, -1.0, 0.5]).unwrap_err(), DmpError::DtNonPositive);
        let zero_slew = |_c: f64| (0.1, 0.0);
        let d0 = DmpPi::new(1.0, c2, rpi, c1, th(), &zero_slew);
        assert_eq!(d0.eval(&[0.0, 1.0, 0.5]).unwrap_err(), DmpError::SlewZero);
    }

    /// A driver with almost no series resistance sees nearly all of its load: Ceff should
    /// approach the total. This is the `DmpCap` end of the spectrum, where
    /// `setCeffAlgorithm` would pick the capacitive algorithm outright.
    #[test]
    fn a_negligible_rpi_leaves_the_load_essentially_lumped() {
        let (c2, _r, c1) = input55();
        let rd = gate_model_rd(th().vth, &linear_gate, c1, c2);
        let r = DmpPi::new(rd, c2, 1e-6, c1, th(), &linear_gate).solve();
        assert!(
            r.ceff > 0.9 * (c1 + c2),
            "with no shielding resistance Ceff {} should approach the total {}",
            r.ceff,
            c1 + c2
        );
    }

    /// ⚠️ `exp2` here is the reference's own fast approximation, not `f64::exp`. Using
    /// `exp` would be MORE accurate than the reference, which is a divergence. Two
    /// properties have to hold: it tracks `exp` loosely, and it hard-zeros below -12.
    #[test]
    fn exp2_is_the_references_approximation_not_exp() {
        assert_eq!(exp2(-13.0), 0.0, "the reference hard-zeros below -12; `exp` does not");
        for x in [-1.0, -0.5, -0.1, 0.0] {
            assert!((exp2(x) - x.exp()).abs() < 1e-3, "exp2({x}) strays too far from exp");
        }
    }
}

#[cfg(test)]
mod shield_tests {
    use super::*;
    fn th() -> Vth {
        Vth { vth: 0.5, vl: 0.2, vh: 0.8, slew_derate: 1.0 }
    }
    fn gate(ceff: f64) -> (f64, f64) {
        (0.10 + 0.35 * ceff, 0.05 + 0.50 * ceff)
    }
    /// ⛔ The exact `input55/X` case, from `report_dcalc` and the `dmp_ceff` debug dump:
    /// Pi `C2=0.1879 Rpi=0.6019k C1=1.0222`, `Rd = 0.467k`, and the gate tables linearised
    /// between the two corners the reference itself printed. The reference solves
    /// **Ceff = 0.6137**.
    #[test]
    fn input55_matches_the_reference_ceff() {
        let (c2, rpi, c1) = (0.1879, 0.6019, 1.0222);
        let rd = 0.467;
        // the two table corners report_dcalc printed for the rise arc
        let gate = |c: f64| {
            (
                0.3805 + (c - 0.4019) * (1.1409 - 0.3805) / (1.5317 - 0.4019),
                0.4086 + (c - 0.4019) * (1.5005 - 0.4086) / (1.5317 - 0.4019),
            )
        };
        let r = solve_driver(rd, c2, rpi, c1, th(), &gate);
        eprintln!("ceff={} fail={:?} delay={} slew={}", r.ceff, r.fail, r.delay, r.slew);
        assert!(
            (r.ceff - 0.6137).abs() < 0.03,
            "Ceff {} should reproduce the reference's 0.6137",
            r.ceff
        );
    }

    /// The per-sink solve, on `input55`'s driver. The reference's own numbers for the
    /// worst sink on that net: wire delay **0.6381** and a sink slew of **1.6616** against
    /// a driver slew of 1.1048 — the wire degrades the edge by half a nanosecond.
    #[test]
    fn the_sink_solve_degrades_the_slew_and_adds_a_wire_delay() {
        let (c2, rpi, c1) = (0.1879, 0.6019, 1.0222);
        let gate = |c: f64| {
            (
                0.3805 + (c - 0.4019) * (1.1409 - 0.3805) / (1.5317 - 0.4019),
                0.4086 + (c - 0.4019) * (1.5005 - 0.4086) / (1.5317 - 0.4019),
            )
        };
        let r = solve_driver(0.467, c2, rpi, c1, th(), &gate);
        let w = r.waveform.expect("a Pi driver hands on a waveform");
        assert!(r.driver_valid, "the driver solve should have converged");
        // the reference's implied tau for that sink, from its own wire delay
        let (delay, slew) = w.load_delay_slew(0.92);
        eprintln!("wire delay={delay} sink slew={slew} (driver slew {})", r.slew);
        assert!(delay > 0.0, "a wire delay must be positive, got {delay}");
        assert!(
            slew > r.slew,
            "the wire must DEGRADE the edge: sink slew {slew} vs driver {}",
            r.slew
        );
    }

    /// ⛔ The degenerate branch is most of a real design, not a corner: an Elmore small
    /// against the driver slew means the wire delay IS the Elmore value and the slew is
    /// the driver's, undegraded. `DmpCap` overrides the whole method with exactly this.
    #[test]
    fn a_small_elmore_takes_the_degenerate_branch() {
        let (c2, rpi, c1) = (0.1879, 0.6019, 1.0222);
        let gate = |c: f64| (0.3805 + 0.673 * c, 0.4086 + 0.966 * c);
        let r = solve_driver(0.467, c2, rpi, c1, th(), &gate);
        let w = r.waveform.unwrap();
        let tiny = r.slew * 1e-4; // under the reference's `drvr_slew * 1e-3` threshold
        assert_eq!(w.load_delay_slew(tiny), (tiny, r.slew));
        assert_eq!(w.load_delay_slew(0.0), (0.0, r.slew), "zero elmore, zero wire delay");
    }

    /// A capacitive driver has no waveform to solve against, so every sink takes
    /// `delay = elmore` and the driver's slew — `DmpCap::loadDelaySlew`.
    #[test]
    fn a_capacitive_driver_degrades_no_slew_at_any_sink() {
        let gate = |c: f64| (0.1 + 0.35 * c, 0.05 + 0.5 * c);
        // rpi = 0 forces `Alg::Cap` through `setCeffAlgorithm`'s own test
        let r = solve_driver(0.467, 0.1879, 0.0, 1.0222, th(), &gate);
        assert_eq!(select_alg(0.467, 0.1879, 0.0, 1.0222), Alg::Cap);
        let w = r.waveform.unwrap();
        assert_eq!(w.load_delay_slew(0.5), (0.5, r.slew));
    }

    /// ⛔ THE TEST THAT RETIRED A WRONG FINDING. `net55`'s worst sink: the reference
    /// reports a wire delay of **0.6381** and a sink slew of **1.6616**, and our Elmore
    /// walk gives that sink **tau = 0.658**.
    ///
    /// Inverting the CLOSED FORM `delay = -tau*ln(1-Vth)` on the reference's 0.6381 implies
    /// tau = 0.921, and for two sessions that was recorded as "our tau is 30 % short".
    /// It never was. The reference does not use the closed form on a gate-driven net; it
    /// uses this crossing solve, and at tau = 0.658 the crossing solve reproduces BOTH of
    /// its numbers. 🔑 **Inverting the wrong law to infer an input is how a correct
    /// component gets blamed.**
    #[test]
    fn our_tau_reproduces_the_reference_sink_through_the_crossing_solve() {
        let (c2, rpi, c1) = (0.1879, 0.6019, 1.0222);
        let gate = |c: f64| {
            (
                0.3805 + (c - 0.4019) * (1.1409 - 0.3805) / (1.5317 - 0.4019),
                0.4086 + (c - 0.4019) * (1.5005 - 0.4086) / (1.5317 - 0.4019),
            )
        };
        let r = solve_driver(0.467, c2, rpi, c1, th(), &gate);
        let w = r.waveform.unwrap();
        let (delay, slew) = w.load_delay_slew(0.658); // our own Elmore for that sink
        assert!(
            (delay - 0.6381).abs() < 0.03,
            "wire delay {delay} should reproduce the reference's 0.6381"
        );
        assert!(
            (slew - 1.6616).abs() < 0.06,
            "sink slew {slew} should reproduce the reference's 1.6616"
        );
    }

    /// ⛔ `ZeroC2` is not exotic: a single lumped C behind a single R reduces to c2 = 0
    /// EXACTLY. Moment matching on one R and one C gives y1 = C, y2 = −R·C², y3 = R²·C³,
    /// hence c1 = C, c2 = 0, rpi = R. A two-element SPEF always lands here, and declining
    /// to solve it cost a real slew degradation on such a net.
    #[test]
    fn a_single_rc_selects_zero_c2_and_still_degrades_the_slew() {
        // 100 fF behind 10 kΩ, driven by a gate with a load-dependent table
        let (c2, rpi, c1) = (0.0, 10.0, 0.100);
        let gate = |c: f64| (0.08 + 13.3 * c, 0.03 + 6.7 * c);
        let rd = gate_model_rd(th().vth, &gate, c1, c2);
        assert_eq!(select_alg(rd, c2, rpi, c1), Alg::ZeroC2, "one R and one C is the ZeroC2 case");
        let r = solve_driver(rd, c2, rpi, c1, th(), &gate);
        assert_eq!(r.ceff, c1, "ZeroC2 does not solve for Ceff, it IS c1");
        assert!(r.driver_valid, "and the waveform solve must converge, got {:?}", r.fail);
        let w = r.waveform.unwrap();
        let (delay, slew) = w.load_delay_slew(1.0); // tau = 10 kΩ × 100 fF = 1 ns
        assert!(delay > 0.0, "wire delay {delay} must be positive");
        assert!(
            slew > r.slew * 1.5,
            "a 1 ns RC must heavily degrade the edge: sink {slew} vs driver {}",
            r.slew
        );
    }

    /// The textbook shielding case: a tiny near capacitance and a large far one behind a
    /// real resistance. Ceff must land much closer to the near cap than to the total,
    /// which is the entire point of an effective capacitance.
    #[test]
    fn a_shielded_far_cap_pulls_ceff_well_below_the_total() {
        let (c2, rpi, c1) = (0.002, 5.0, 0.200); // pF, kΩ, pF
        let rd = gate_model_rd(th().vth, &gate, c1, c2);
        let r = solve_driver(rd, c2, rpi, c1, th(), &gate);
        eprintln!("rd={rd} alg={:?} ceff={} total={}", select_alg(rd, c2, rpi, c1), r.ceff, c1 + c2);
        assert!(
            r.ceff < 0.5 * (c1 + c2),
            "Ceff {} should be well under the total {} when 200 fF sits behind 5 kΩ",
            r.ceff,
            c1 + c2
        );
    }
}
