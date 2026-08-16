package org.gnss.ignav.fusion;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition;
import org.gnss.ignav.contract.GnssObservation;
import org.gnss.ignav.contract.StateCorrection;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class EkfFusion {

    private static final Logger logger = LoggerFactory.getLogger(EkfFusion.class);

    private int nx;
    private DMatrixRMaj x;
    private DMatrixRMaj P;
    private boolean initialized;
    private double chi2Threshold;
    private double lastInnovationRatio;

    public EkfFusion() {
        this.nx = 15;
        this.x = new DMatrixRMaj(nx, 1);
        this.P = new DMatrixRMaj(nx, nx);
        this.initialized = false;
        this.chi2Threshold = 0.01;
        this.lastInnovationRatio = 0.0;
    }

    public void init(int stateDim, double[] initState, double[] initCov) {
        this.nx = stateDim;
        this.x = new DMatrixRMaj(nx, 1);
        this.P = new DMatrixRMaj(nx, nx);

        if (initState != null && initState.length >= nx) {
            for (int i = 0; i < nx; i++) {
                x.set(i, 0, initState[i]);
            }
        }

        if (initCov != null && initCov.length >= nx * nx) {
            for (int i = 0; i < nx; i++) {
                for (int j = 0; j < nx; j++) {
                    P.set(i, j, initCov[i * nx + j]);
                }
            }
        } else {
            CommonOps_DDRM.setIdentity(P);
            CommonOps_DDRM.scale(1e-6, P);
        }

        initialized = true;
        logger.info("EKF fusion initialized, nx={}", nx);
    }

    public StateCorrection lcUpdate(GnssObservation obs) {
        if (!initialized || obs == null || obs.getNm() <= 0) {
            return null;
        }

        int nm = obs.getNm();
        int obsNx = obs.getNx();
        if (obsNx != nx) {
            logger.warn("Observation nx={} != filter nx={}", obsNx, nx);
            return null;
        }

        double[] vArr = obs.getV();
        double[] HArr = obs.getH();
        double[] RArr = obs.getR();
        if (vArr == null || HArr == null || RArr == null) {
            return null;
        }

        DMatrixRMaj v = new DMatrixRMaj(nm, 1);
        DMatrixRMaj H = new DMatrixRMaj(nm, nx);
        DMatrixRMaj R = new DMatrixRMaj(nm, nm);

        for (int i = 0; i < nm; i++) {
            v.set(i, 0, vArr[i]);
        }
        for (int i = 0; i < nm; i++) {
            for (int j = 0; j < nx; j++) {
                H.set(i, j, HArr[i * nx + j]);
            }
        }
        for (int i = 0; i < nm; i++) {
            for (int j = 0; j < nm; j++) {
                R.set(i, j, RArr[i * nm + j]);
            }
        }

        DMatrixRMaj PHt = new DMatrixRMaj(nx, nm);
        CommonOps_DDRM.multTransB(P, H, PHt);

        DMatrixRMaj S = new DMatrixRMaj(nm, nm);
        CommonOps_DDRM.mult(H, PHt, S);
        CommonOps_DDRM.addEquals(S, R);

        CholeskyDecomposition<DMatrixRMaj> chol = DecompositionFactory_DDRM.chol(nm, true);
        if (!chol.decompose(S)) {
            logger.warn("Cholesky decomposition failed, skipping LC update");
            return null;
        }

        DMatrixRMaj Sinv = new DMatrixRMaj(nm, nm);
        if (!CommonOps_DDRM.invert(S, Sinv)) {
            logger.warn("S inversion failed, skipping LC update");
            return null;
        }

        DMatrixRMaj K = new DMatrixRMaj(nx, nm);
        CommonOps_DDRM.mult(PHt, Sinv, K);

        DMatrixRMaj dx = new DMatrixRMaj(nx, 1);
        CommonOps_DDRM.mult(K, v, dx);

        DMatrixRMaj KH = new DMatrixRMaj(nx, nx);
        CommonOps_DDRM.mult(K, H, KH);

        DMatrixRMaj I = new DMatrixRMaj(nx, nx);
        CommonOps_DDRM.setIdentity(I);

        DMatrixRMaj IKH = new DMatrixRMaj(nx, nx);
        CommonOps_DDRM.subtract(I, KH, IKH);

        DMatrixRMaj dP = new DMatrixRMaj(nx, nx);
        CommonOps_DDRM.mult(IKH, P, dP);

        DMatrixRMaj vSv = new DMatrixRMaj(1, 1);
        DMatrixRMaj vT = new DMatrixRMaj(1, nm);
        CommonOps_DDRM.transpose(v, vT);
        CommonOps_DDRM.mult(vT, Sinv, vSv);

        double innovationRatio = vSv.get(0, 0) / nm;
        lastInnovationRatio = innovationRatio;

        if (innovationRatio > chi2Threshold * 10) {
            logger.warn("Innovation ratio too large: {}, possible outlier", innovationRatio);
            return null;
        }

        double[] dxArr = new double[nx];
        double[] dPArr = new double[nx * nx];
        for (int i = 0; i < nx; i++) {
            dxArr[i] = dx.get(i, 0);
        }
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < nx; j++) {
                dPArr[i * nx + j] = dP.get(i, j) - P.get(i, j);
            }
        }

        CommonOps_DDRM.addEquals(x, dx);
        P.setTo(dP);

        symmetrize(P);

        return new StateCorrection(dxArr, dPArr);
    }

    public StateCorrection tcUpdate(GnssObservation obs) {
        if (!initialized || obs == null || obs.getNm() <= 0) {
            return null;
        }

        return lcUpdate(obs);
    }

    public void predict(double dt, double[] Qdiag) {
        if (!initialized) return;

        DMatrixRMaj F = new DMatrixRMaj(nx, nx);
        buildStateTransitionF(F, dt);

        DMatrixRMaj Phi = new DMatrixRMaj(nx, nx);
        CommonOps_DDRM.setIdentity(Phi);

        if (dt > 0 && dt < 1.0) {
            DMatrixRMaj Fdt = new DMatrixRMaj(nx, nx);
            DMatrixRMaj Fdt2 = new DMatrixRMaj(nx, nx);
            CommonOps_DDRM.scale(dt, F, Fdt);
            CommonOps_DDRM.mult(Fdt, Fdt, Fdt2);
            CommonOps_DDRM.scale(0.5, Fdt2);
            CommonOps_DDRM.addEquals(Phi, Fdt);
            CommonOps_DDRM.addEquals(Phi, Fdt2);
        }

        DMatrixRMaj Q = new DMatrixRMaj(nx, nx);
        if (Qdiag != null && Qdiag.length >= nx) {
            for (int i = 0; i < nx; i++) {
                Q.set(i, i, Qdiag[i] * dt);
            }
        } else {
            for (int i = 0; i < 3; i++) Q.set(i, i, 0.01 * dt);
            for (int i = 3; i < 6; i++) Q.set(i, i, 0.001 * dt);
            for (int i = 6; i < 9; i++) Q.set(i, i, 0.0001 * dt);
        }

        DMatrixRMaj PhiT = new DMatrixRMaj(nx, nx);
        CommonOps_DDRM.transpose(Phi, PhiT);

        DMatrixRMaj PhiP = new DMatrixRMaj(nx, nx);
        CommonOps_DDRM.mult(Phi, P, PhiP);

        DMatrixRMaj PhiPPhiT = new DMatrixRMaj(nx, nx);
        CommonOps_DDRM.mult(PhiP, PhiT, PhiPPhiT);

        CommonOps_DDRM.addEquals(PhiPPhiT, Q);
        P.setTo(PhiPPhiT);
    }

    private void buildStateTransitionF(DMatrixRMaj F, double dt) {
        F.zero();

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                F.set(i, 3 + j, 0.0);
            }
        }

        for (int i = 0; i < 3; i++) {
            F.set(3 + i, 6 + i, 1.0);
        }

        if (nx >= 15) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    F.set(i, 12 + j, -1.0);
                }
            }
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    F.set(3 + i, 9 + j, -1.0);
                }
            }
        }
    }

    public void resetCovariance(double[] std) {
        if (std == null || std.length < nx) return;
        P.zero();
        for (int i = 0; i < nx; i++) {
            P.set(i, i, std[i] * std[i]);
        }
    }

    public double[] getState() {
        double[] result = new double[nx];
        for (int i = 0; i < nx; i++) {
            result[i] = x.get(i, 0);
        }
        return result;
    }

    public double[] getCovariance() {
        double[] result = new double[nx * nx];
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < nx; j++) {
                result[i * nx + j] = P.get(i, j);
            }
        }
        return result;
    }

    public double getCovTrace() {
        double trace = 0.0;
        for (int i = 0; i < nx; i++) {
            trace += P.get(i, i);
        }
        return trace;
    }

    public double getLastInnovationRatio() {
        return lastInnovationRatio;
    }

    public int getStateDimension() {
        return nx;
    }

    public boolean isInitialized() {
        return initialized;
    }

    private void symmetrize(DMatrixRMaj mat) {
        int n = mat.numRows;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double avg = (mat.get(i, j) + mat.get(j, i)) / 2.0;
                mat.set(i, j, avg);
                mat.set(j, i, avg);
            }
        }
    }
}