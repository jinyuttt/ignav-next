package org.gnss.ignav.ins.smooth;

import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.InsState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsRts {

    private static final Logger logger = LoggerFactory.getLogger(InsRts.class);

    private InsRts() {}

    public static void rts(InsState[] insFwd, InsState[] insSmooth, int n, double[] Phi, double[] Q) {
        int i, j, nx;
        double[] Pp, Ppinv, K, Ps;

        if (n <= 0 || insFwd == null) return;

        nx = insFwd[0].nx;
        if (nx <= 0) return;

        Pp = new double[nx * nx];
        Ppinv = new double[nx * nx];
        K = new double[nx * nx];
        Ps = new double[nx * nx];

        if (insSmooth[n - 1] == null) insSmooth[n - 1] = new InsState();
        insSmooth[n - 1].nx = nx;
        if (insSmooth[n - 1].x == null) insSmooth[n - 1].x = new double[nx];
        if (insSmooth[n - 1].P == null) insSmooth[n - 1].P = new double[nx * nx];
        System.arraycopy(insFwd[n - 1].x, 0, insSmooth[n - 1].x, 0, nx);
        InsMath.matcpy(insSmooth[n - 1].P, insFwd[n - 1].P, nx, nx);

        for (i = n - 2; i >= 0; i--) {
            if (insFwd[i] == null || insFwd[i + 1] == null) continue;

            double[] PhiT = new double[nx * nx];
            InsMath.matt(Phi, nx, nx, PhiT);

            InsMath.matmul("NN", nx, nx, nx, 1.0, Phi, insFwd[i].P, 0.0, Pp);
            InsMath.matmul("NN", nx, nx, nx, 1.0, Pp, PhiT, 0.0, Pp);
            for (j = 0; j < nx * nx; j++) Pp[j] += Q[j];

            InsMath.matcpy(Ppinv, Pp, nx, nx);
            if (InsMath.matinv(Ppinv, nx) != 0) {
                logger.warn("RTS: matrix inversion failed at index {}", i);
                continue;
            }

            InsMath.matmul("NN", nx, nx, nx, 1.0, insFwd[i].P, PhiT, 0.0, K);
            InsMath.matmul("NN", nx, nx, nx, 1.0, K, Ppinv, 0.0, K);

            double[] dx = new double[nx];
            for (j = 0; j < nx; j++)
                dx[j] = insSmooth[i + 1].x[j] - insFwd[i + 1].x[j];

            if (insSmooth[i] == null) insSmooth[i] = new InsState();
            insSmooth[i].nx = nx;
            if (insSmooth[i].x == null) insSmooth[i].x = new double[nx];
            InsMath.matmul("NN", nx, 1, nx, 1.0, K, dx, 0.0, insSmooth[i].x);
            for (j = 0; j < nx; j++)
                insSmooth[i].x[j] += insFwd[i].x[j];

            double[] dP = new double[nx * nx];
            double[] I = InsMath.eye(nx);
            InsMath.matmul("NN", nx, nx, nx, 1.0, K, Phi, 0.0, dP);
            for (j = 0; j < nx * nx; j++) dP[j] = I[j] - dP[j];

            if (insSmooth[i].P == null) insSmooth[i].P = new double[nx * nx];
            InsMath.matmul("NN", nx, nx, nx, 1.0, dP, insFwd[i].P, 0.0, Ps);
            double[] Kt = new double[nx * nx];
            InsMath.matt(K, nx, nx, Kt);
            double[] tmp = new double[nx * nx];
            InsMath.matmul("NN", nx, nx, nx, 1.0, K, insSmooth[i + 1].P, 0.0, tmp);
            InsMath.matmul("NN", nx, nx, nx, 1.0, tmp, Kt, 0.0, tmp);
            for (j = 0; j < nx * nx; j++)
                insSmooth[i].P[j] = Ps[j] + tmp[j] - Pp[j];
        }
    }
}