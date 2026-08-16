package org.gnss.ignav.ins.mech;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.GTime;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsInitRt {

    private static final Logger logger = LoggerFactory.getLogger(InsInitRt.class);

    private static final double MINVEL = 5.0;
    private static final double MAXGYRO = 30.0 * IgnavConstants.D2R;

    private InsInitRt() {}

    public static int insinirt(double[] pos, double[] vel, GTime time, Imud imu, InsOpt opt, InsState ins) {
        double[] vr = new double[3];

        ins.stat = IgnavConstants.INSS_INIT;

        if (pos == null || vel == null) {
            logger.warn("no position/velocity data to initial");
            return 0;
        }

        InsMath.matcpy(vr, vel, 1, 3);
        if (InsMath.norm(vr, 3) == 0.0) {
            logger.warn("velocity is zero");
            return 0;
        }
        if (InsMath.norm(imu.gyro, 3) > MAXGYRO || InsMath.norm(vr, 3) < MINVEL) {
            logger.warn("gyro too large or velocity too small");
            return 0;
        }

        if (ant2inins(time, pos, vel, opt, imu, ins) == 0) {
            logger.warn("ant2inins fail");
            return 0;
        }
        ins.time = new GTime(time);

        InsMech.updateInsStateN(ins);

        logger.info("initial ins state ok");
        return 1;
    }

    public static int insinidualant(double[] pos, double[] vel, double[] rpy,
                                     GTime time, Imud imu, InsOpt opt, InsState ins) {
        double[] Cne = new double[9], Cvn = new double[9];
        double[] RyArr = new double[9], RzArr = new double[9];
        double[] posllh = new double[3];

        ins.stat = IgnavConstants.INSS_INIT;

        if (pos == null || vel == null || rpy == null) {
            logger.warn("no position/velocity/attitude data to initial");
            return 0;
        }

        InsMath.ecef2pos(pos, posllh);
        InsMath.ned2xyz(posllh, Cne);

        InsMath.Ry(-rpy[1], RyArr);
        InsMath.Rz(-rpy[2], RzArr);
        InsMath.matmul("NN", 3, 3, 3, 1.0, RzArr, RyArr, 0.0, Cvn);

        InsMath.matmul33("NNT", Cne, Cvn, ins.Cvb, 3, 3, 3, 3, ins.Cbe);
        InsMath.gapv2ipv(pos, vel, ins.Cbe, opt.lever, imu, ins.re, ins.ve);

        InsMech.updateInsStateN(ins);

        ins.time = new GTime(time);
        logger.info("initial ins state from dual antenna ok");
        return 1;
    }

    public static int ant2inins(GTime time, double[] rr, double[] vr, InsOpt opt, Imud imu, InsState ins) {
        double[] llh = new double[3];
        double[] vn = new double[3];
        double[] C = new double[9];
        double[] rpy = new double[3];

        InsMath.ecef2pos(rr, llh);
        InsMath.ned2xyz(llh, C);
        InsMath.matmul3v("TN", C, vr, vn);

        InsMath.matcpy(ins.rn, llh, 1, 3);
        InsMath.matcpy(ins.vn, vn, 1, 3);

        rpy[2] = InsMath.vel2head(vn);
        InsMath.rpy2dcm(rpy, C);
        InsMath.matt(C, 3, 3, ins.Cbn);

        InsMath.ned2xyz(llh, C);
        InsMath.matmul("NN", 3, 3, 3, 1.0, C, ins.Cbn, 0.0, ins.Cbe);

        InsMath.gapv2ipv(rr, vr, ins.Cbe, opt.lever, imu, ins.re, ins.ve);
        return 1;
    }
}