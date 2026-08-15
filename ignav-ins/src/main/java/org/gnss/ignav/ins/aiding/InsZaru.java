package org.gnss.ignav.ins.aiding;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.gnss.ignav.ins.mech.InsAlignMech;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsZaru {

    private static final Logger logger = LoggerFactory.getLogger(InsZaru.class);

    private static final double MAXVEL_ZARU = 0.1;
    private static final double MAXGYRO_ZARU = 5.0 * IgnavConstants.D2R;
    private static final double VARARE = InsMath.SQR(1.0 * IgnavConstants.D2R);
    private static final int MINZAC = 100;

    private static int nz = 0;

    private InsZaru() {}

    public static int zaru(InsState ins, InsOpt opt, Imud imu, int flag) {
        int info = 0;
        int nx = ins.nx;

        flag &= (nz++ > MINZAC ? 1 : 0);
        if (nz > MINZAC) nz = 0;

        if (flag == 0 || opt.bgopt != IgnavConstants.INS_BGEST) return 0;

        double[] x = new double[nx];
        double[] v = new double[3];
        double[] H = new double[3 * nx];
        double[] R = new double[3 * 3];
        double[] I = {-1, 0, 0, 0, -1, 0, 0, 0, -1};

        InsAlignMech.asiBlkMat(H, 3, nx, I, 3, 3, 0, InsStateIdx.xiBg(opt));

        R[0] = R[4] = R[8] = VARARE;

        v[0] = -imu.gyro[0];
        v[1] = -imu.gyro[1];
        v[2] = -imu.gyro[2];

        if (InsMath.norm(v, 3) < MAXGYRO_ZARU && InsMath.norm(ins.ve, 3) < MAXVEL_ZARU) {
            info = InsAlignMech.filter(x, ins.P, H, v, R, nx, 3);

            if (info != 0) {
                logger.warn("zero angular rate update filter error");
                info = 0;
            } else {
                ins.stat = IgnavConstants.INSS_ZARU;
                info = 1;
                InsNhc.clp(ins, opt, x);
                logger.info("zero angular rate update ok");
            }
        }
        return info;
    }
}