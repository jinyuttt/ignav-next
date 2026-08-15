package org.gnss.ignav.ins.mech;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.GTime;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsBackMech {

    private static final Logger logger = LoggerFactory.getLogger(InsBackMech.class);

    private static final double MAXDT = 60.0;

    private InsBackMech() {}

    private static void savepins(InsState ins, Imud data) {
        InsMath.matcpy(ins.omgbp, ins.omgb, 1, 3);
        InsMath.matcpy(ins.fbp, ins.fb, 1, 3);
        InsMath.matcpy(ins.pins, ins.re, 1, 3);
        InsMath.matcpy(ins.pins, ins.ve, 1, 3);
        InsMath.matcpy(ins.pCbe, ins.Cbe, 3, 3);
    }

    private static void getqbn(InsState ins, double[] qbn) {
        double[] pos = new double[3];
        double[] Cne = new double[9];
        double[] Cbn = new double[9];
        InsMath.ecef2pos(ins.re, pos);
        InsMath.ned2xyz(pos, Cne);
        InsMath.matmul3("TN", Cne, ins.Cbe, Cbn);
        InsMath.dcm2quat(Cbn, qbn);
    }

    private static void updateBn(InsState ins, double[] qbn, double[] vn, double[] Cen, double h) {
        double[] Cbn = new double[9];
        double[] rn = new double[3];
        InsMath.quat2dcm(qbn, Cbn);
        InsMath.matcpy(ins.Cbn, Cbn, 3, 3);
        InsMath.matcpy(ins.vn, vn, 1, 3);
        InsMath.matmul("TN", 3, 3, 3, 1.0, Cen, Cbn, 0.0, ins.Cbe);
        InsMath.matmul("TN", 3, 1, 3, 1.0, Cen, vn, 0.0, ins.ve);
        rn[0] = Math.acos(Cen[6]);
        rn[1] = Math.acos(Cen[4]);
        rn[2] = h;
        InsMath.matcpy(ins.rn, rn, 1, 3);
        InsMath.pos2ecef(rn, ins.re);
        InsMath.getaccl(ins.fb, ins.Cbe, ins.re, ins.ve, ins.ae);
    }

    public static int updateinsbn(InsOpt insopt, InsState ins, Imud data) {
        double[] wenN = new double[3], wieN = new double[3];
        double g, dt;
        double[] vn = new double[3], dv = new double[3], da = new double[3];
        double[] dv1 = new double[3], dv2 = new double[3], qbn = new double[4];
        double[] w = new double[3], rn = new double[3];
        double[] qb = new double[4], qn = new double[4], q = new double[4], vmid = new double[3];
        double[] Cen = new double[9], Cne = new double[9], Cn = new double[9];
        double h;
        int i;

        InsMech.updateInsStateN(ins);

        savepins(ins, data);

        dt = -GTime.timeDiff(data.time, ins.time);
        if (dt > MAXDT || Math.abs(dt) < 1E-6) {
            ins.dt = GTime.timeDiff(data.time, ins.time);
            ins.ptime = new GTime(ins.time);
            ins.time = new GTime(data.time);
            logger.warn("time difference too large: {}s", dt);
            return 0;
        }

        for (i = 0; i < 3; i++) {
            ins.omgb0[i] = data.gyro[i];
            ins.fb0[i] = data.accl[i];
            if (insopt.exinserr != 0) {
                InsMech.insErrmodel(data.accl, data.gyro, ins.fb, ins.omgb, ins);
            } else {
                ins.omgb[i] = data.gyro[i] - ins.bg[i];
                ins.fb[i] = data.accl[i] - ins.ba[i];
            }
        }

        InsMath.ecef2pos(ins.re, rn);
        InsMath.getvn(ins, vn);

        double[] gArr = new double[1];
        InsMech.geoparam(rn, vn, wenN, wieN, gArr, null, null);
        g = gArr[0];

        for (i = 0; i < 3; i++) {
            dv[i] = ins.fb[i] * dt;
            da[i] = -ins.omgb[i] * dt;
        }

        getqbn(ins, qbn);
        InsMath.quatrot(qbn, dv, 0, dv1);

        for (i = 0; i < 3; i++) w[i] = 2.0 * wieN[i] + wenN[i];
        InsMath.cross3(vn, w, dv2);
        dv2[2] += g;

        for (i = 0; i < 3; i++) dv2[i] *= dt;
        for (i = 0; i < 3; i++) {
            ins.vn[i] = vn[i] - dv1[i] - dv2[i];
            ins.an[i] = (dv1[i] + dv2[i]) / dt;
        }

        InsMath.rvec2quat(da, qb);
        InsMath.quatmul(qbn, qb, q);

        for (i = 0; i < 3; i++) w[i] = (wenN[i] + wieN[i]) * dt;
        InsMath.rvec2quat(w, qn);
        InsMath.quatmul(qn, q, qbn);

        for (i = 0; i < 3; i++) vmid[i] = (vn[i] + ins.vn[i]) / 2.0;
        InsMech.geoparam(rn, vmid, wenN, null, null, null, null);
        for (i = 0; i < 3; i++) w[i] = wenN[i] * dt;
        InsMath.rot2dcm(w, Cn);
        InsMath.ned2xyz(rn, Cne);
        InsMath.matmul("NT", 3, 3, 3, 1.0, Cn, Cne, 0.0, Cen);
        h = rn[2] + vmid[2] * dt;

        updateBn(ins, qbn, ins.vn, Cen, h);

        ins.dt = GTime.timeDiff(data.time, ins.time);
        ins.ptime = new GTime(ins.time);
        ins.time = new GTime(data.time);
        ins.stat = IgnavConstants.INSS_MECH;

        return 1;
    }
}