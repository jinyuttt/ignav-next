package org.gnss.ignav.ins.aiding;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.gnss.ignav.ins.mech.InsAlignMech;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsMagnetometer {

    private static final Logger logger = LoggerFactory.getLogger(InsMagnetometer.class);

    private static final double VARMAG = InsMath.SQR(0.1 * IgnavConstants.D2R);

    private InsMagnetometer() {}

    public static int magupd(InsState ins, InsOpt opt, double[] magMeas, double[] magRef) {
        int nx = ins.nx;
        int info = 0;
        int IA = InsStateIdx.xiA(opt);
        int i;

        double[] dcm = new double[9];
        double[] H = new double[2 * nx];
        double[] R = new double[2 * 2];
        double[] v = new double[2];
        double[] x = new double[nx];

        double[] mn = new double[3];
        InsMath.matmul3v("TN", ins.Cbe, magMeas, mn);

        double heading = Math.atan2(mn[1], mn[0]);
        double refHeading = Math.atan2(magRef[1], magRef[0]);

        v[0] = refHeading - heading;
        if (v[0] > Math.PI) v[0] -= 2.0 * Math.PI;
        if (v[0] < -Math.PI) v[0] += 2.0 * Math.PI;

        H[IA + 2] = 1.0;
        R[0] = VARMAG;

        info = InsAlignMech.filter(x, ins.P, H, v, R, nx, 1);

        if (info != 0) {
            logger.warn("magnetometer update filter error");
            return 0;
        } else {
            ins.stat = 6;
            info = 1;
            InsNhc.clp(ins, opt, x);
            logger.info("magnetometer update ok");
        }
        return info;
    }
}