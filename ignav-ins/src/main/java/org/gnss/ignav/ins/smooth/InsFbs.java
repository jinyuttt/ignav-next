package org.gnss.ignav.ins.smooth;

import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.InsState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsFbs {

    private static final Logger logger = LoggerFactory.getLogger(InsFbs.class);

    private InsFbs() {}

    public static void fbs(InsState[] insFwd, InsState[] insBwd, InsState[] insSmooth, int n) {
        int i, j, nx;
        double[] Pf, Pb, Ps, Pfinv, Pbinv;

        if (n <= 0 || insFwd == null || insBwd == null) return;

        nx = insFwd[0].nx;
        if (nx <= 0) return;

        Pf = new double[nx * nx];
        Pb = new double[nx * nx];
        Ps = new double[nx * nx];
        Pfinv = new double[nx * nx];
        Pbinv = new double[nx * nx];

        for (i = 0; i < n; i++) {
            if (insFwd[i] == null || insBwd[n - 1 - i] == null) continue;

            InsMath.matcpy(Pf, insFwd[i].P, nx, nx);
            InsMath.matcpy(Pb, insBwd[n - 1 - i].P, nx, nx);

            InsMath.matcpy(Pfinv, Pf, nx, nx);
            InsMath.matcpy(Pbinv, Pb, nx, nx);

            if (InsMath.matinv(Pfinv, nx) != 0 || InsMath.matinv(Pbinv, nx) != 0) {
                logger.warn("matrix inversion failed for FBS at index {}", i);
                continue;
            }

            for (j = 0; j < nx * nx; j++)
                Ps[j] = Pfinv[j] + Pbinv[j];

            if (InsMath.matinv(Ps, nx) != 0) {
                logger.warn("smooth covariance inversion failed at index {}", i);
                continue;
            }

            if (insSmooth[i] == null) insSmooth[i] = new InsState();
            insSmooth[i].nx = nx;

            double[] tmpF = new double[nx];
            double[] tmpB = new double[nx];
            double[] xf = new double[nx];
            double[] xb = new double[nx];

            System.arraycopy(insFwd[i].x, 0, xf, 0, nx);
            System.arraycopy(insBwd[n - 1 - i].x, 0, xb, 0, nx);

            InsMath.matmul("NN", nx, 1, nx, 1.0, Pfinv, xf, 0.0, tmpF);
            InsMath.matmul("NN", nx, 1, nx, 1.0, Pbinv, xb, 0.0, tmpB);

            if (insSmooth[i].x == null) insSmooth[i].x = new double[nx];
            for (j = 0; j < nx; j++)
                insSmooth[i].x[j] = tmpF[j] + tmpB[j];
            InsMath.matmul("NN", nx, 1, nx, 1.0, Ps, insSmooth[i].x, 0.0, tmpF);
            System.arraycopy(tmpF, 0, insSmooth[i].x, 0, nx);

            if (insSmooth[i].P == null) insSmooth[i].P = new double[nx * nx];
            InsMath.matcpy(insSmooth[i].P, Ps, nx, nx);
        }
    }
}