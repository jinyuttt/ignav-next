package org.gnss.ignav.ins.aiding;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.gnss.ignav.ins.mech.InsAlignMech;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsZvu {

    private static final Logger logger = LoggerFactory.getLogger(InsZvu.class);

    private static final double MAXVEL_ZVU = 0.1;
    private static final double MAXGYRO_ZVU = 10.0 * IgnavConstants.D2R;
    private static final double VARVEL = InsMath.SQR(0.05);
    private static final int MINZC = 15;

    private static int nz = 0;

    private InsZvu() {}

    public static int zvu(InsState ins, InsOpt opt, Imud imu, int flag) {
        int nx = ins.nx;
        int info = 0;

        flag &= (nz++ > MINZC ? 1 : 0);
        if (nz > MINZC) nz = 0;

        if (flag == 0) return info;

        double[] x = new double[nx];
        double[] H = new double[3 * nx];
        double[] R = new double[3 * 3];
        double[] v = new double[3];
        double[] I = {-1, 0, 0, 0, -1, 0, 0, 0, -1};

        InsAlignMech.asiBlkMat(H, 3, nx, I, 3, 3, 0, 3);

        R[0] = R[4] = R[8] = VARVEL;

        v[0] = ins.ve[0];
        v[1] = ins.ve[1];
        v[2] = ins.ve[2];

        if (InsMath.norm(v, 3) < MAXVEL_ZVU && InsMath.norm(imu.gyro, 3) < MAXGYRO_ZVU) {
            info = InsAlignMech.filter(x, ins.P, H, v, R, nx, 3);

            if (info != 0) {
                logger.warn("zero velocity update filter error");
                info = 0;
            } else {
                ins.stat = IgnavConstants.INSS_ZVU;
                info = 1;
                InsNhc.clp(ins, opt, x);
                logger.info("zero velocity update ok");
            }
        }
        return info;
    }
}