package org.gnss.ignav.ins.aiding;

import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.Odod;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsStaticDetect {

    private static final Logger logger = LoggerFactory.getLogger(InsStaticDetect.class);

    private static double odoDt = 0.0;
    private static double odoDr = 0.0;

    private InsStaticDetect() {}

    public static int detstaticGlrt(Imud[] imu, int n, InsOpt opt, double[] pos) {
        int i, j;
        double[] gn = new double[3], ym = new double[3], tmp = new double[3];
        double T = 0, sg = opt.zvopt.sigG, sa = opt.zvopt.sigA;

        InsMath.gravityNed(pos, gn);

        for (i = 0; i < 3; i++) {
            for (j = 0; j < n; j++) ym[i] += imu[j].accl[i];
            ym[i] /= n;
        }
        for (i = 0; i < n; i++) {
            for (j = 0; j < 3; j++)
                tmp[j] = imu[i].accl[j] - InsMath.norm(gn, 3) / InsMath.norm(ym, 3) * ym[j];
            T += InsMath.SQR(InsMath.norm(imu[i].gyro, 3)) / InsMath.SQR(sg)
               + InsMath.SQR(InsMath.norm(tmp, 3)) / InsMath.SQR(sa);
        }
        T /= n;

        logger.debug("GLRT T={}, gamma={}", T, opt.zvopt.gamma[0]);
        return T < opt.zvopt.gamma[0] ? 1 : 0;
    }

    public static int detstaticMv(Imud[] imu, int n, InsOpt opt) {
        int i, j;
        double[] ym = new double[3], tmp = new double[3];
        double T = 0.0;

        for (i = 0; i < 3; i++) {
            for (j = 0; j < n; j++) ym[i] += imu[j].accl[i];
            ym[i] /= n;
        }
        for (i = 0; i < n; i++) {
            for (j = 0; j < 3; j++) tmp[j] = imu[i].accl[j] - ym[j];
            T += InsMath.SQR(InsMath.norm(tmp, 3));
        }
        T /= (InsMath.SQR(opt.zvopt.sigA) * n);

        logger.debug("MV T={}, gamma={}", T, opt.zvopt.gamma[1]);
        return T < opt.zvopt.gamma[1] ? 1 : 0;
    }

    public static int detstaticMag(Imud[] imu, int n, InsOpt opt, double[] pos) {
        int i;
        double[] gn = new double[3];
        double sa2 = InsMath.SQR(opt.zvopt.sigA), T;

        InsMath.gravityNed(pos, gn);

        for (T = 0.0, i = 0; i < n; i++) {
            T += InsMath.SQR(InsMath.norm(gn, 3) - InsMath.norm(imu[i].accl, 3));
        }
        T /= (sa2 * n);

        logger.debug("MAG T={}, gamma={}", T, opt.zvopt.gamma[2]);
        return T < opt.zvopt.gamma[2] ? 1 : 0;
    }

    public static int detstaticAre(Imud[] imu, int n, InsOpt opt) {
        int i;
        double T = 0.0, sg2 = InsMath.SQR(opt.zvopt.sigG);

        for (i = 0; i < n; i++) T += InsMath.SQR(InsMath.norm(imu[i].gyro, 3));
        T /= (sg2 * n);

        logger.debug("ARE T={}, gamma={}", T, opt.zvopt.gamma[3]);
        return T < opt.zvopt.gamma[3] ? 1 : 0;
    }

    public static int detstaticOdo(InsOpt opt, Odod odo) {
        odoDt += odo.dt;
        odoDr += odo.dr;

        if (odoDt > (opt.zvopt.odt == 0.0 ? 0.1 : opt.zvopt.odt)) {
            if (odoDr / odoDt == 0.0) {
                logger.debug("detect zero velocity by using odometry");
                odoDr = odoDt = 0.0;
                return 1;
            }
            odoDr = odoDt = 0.0;
        }
        return 0;
    }
}