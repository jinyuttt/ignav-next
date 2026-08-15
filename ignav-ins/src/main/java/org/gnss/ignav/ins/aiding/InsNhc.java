package org.gnss.ignav.ins.aiding;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.gnss.ignav.ins.mech.InsAlignMech;
import org.gnss.ignav.ins.mech.InsMech;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsNhc {

    private static final Logger logger = LoggerFactory.getLogger(InsNhc.class);

    private static final double MAXVEL_NHC = 0.1;
    private static final double VARVEL_NHC = InsMath.SQR(0.05);

    private InsNhc() {}

    public static void clp(InsState ins, InsOpt opt, double[] x) {
        int i;
        int iba = InsStateIdx.xiBa(opt);
        int nba = InsStateIdx.xnBa(opt);
        int ibg = InsStateIdx.xiBg(opt);
        int nbg = InsStateIdx.xnBg(opt);
        int isg = InsStateIdx.xiSg(opt);
        int nsg = InsStateIdx.xnSg(opt);
        int isa = InsStateIdx.xiSa(opt);
        int nsa = InsStateIdx.xnSa(opt);

        if (x[0] != 0.0) {
            InsMech.corratt(x, ins.Cbe);
        }

        ins.ve[0] -= x[InsStateIdx.xiV(opt) + 0];
        ins.ve[1] -= x[InsStateIdx.xiV(opt) + 1];
        ins.ve[2] -= x[InsStateIdx.xiV(opt) + 2];

        ins.re[0] -= x[InsStateIdx.xiP(opt) + 0];
        ins.re[1] -= x[InsStateIdx.xiP(opt) + 1];
        ins.re[2] -= x[InsStateIdx.xiP(opt) + 2];

        if (nba > 0 && x[iba] != 0.0) {
            for (i = 0; i < 3; i++) ins.ba[i] += x[iba + i];
        }
        if (nbg > 0 && x[ibg] != 0.0) {
            for (i = 0; i < 3; i++) ins.bg[i] += x[ibg + i];
        }
        if (nsg > 0 && x[isg] != 0.0) {
            for (i = isg; i < isg + nsg; i++)
                ins.Mg[i - isg + (i - isg) * 3] += x[i];
        }
        if (nsa > 0 && x[isa] != 0.0) {
            for (i = isa; i < isa + nsa; i++)
                ins.Ma[i - isa + (i - isa) * 3] += x[i];
        }

        double[] fibc = new double[3];
        double[] omgbc = new double[3];
        InsMech.insErrmodel2(ins.fb0, ins.omgb0, ins.Ma, ins.Mg, ins.ba, ins.bg, ins.Gg, fibc, omgbc);
        InsMath.getaccl(fibc, ins.Cbe, ins.re, ins.ve, ins.ae);
    }

    private static int bldnhc(InsOpt opt, Imud imu, double[] Cbe, double[] ve, int nx, double[] v, double[] H, double[] R) {
        int i, nv;
        int IA = InsStateIdx.xiA(opt);
        int IV = InsStateIdx.xiV(opt);
        double[] C = new double[9];
        double[] T = new double[9];
        double[] vb = new double[3];
        double[] r = new double[2];

        InsMath.matmul3v("TN", Cbe, ve, vb);

        InsMath.skewsym3(ve, C);
        InsMath.matmul("TN", 3, 3, 3, 1.0, Cbe, C, 0.0, T);
        double[] CT = new double[9];
        InsMath.matt(Cbe, 3, 3, CT);
        InsMath.matcpy(C, CT, 3, 3);

        for (nv = 0, i = 1; i < 3; i++) {
            if (Math.abs(vb[i]) > MAXVEL_NHC) {
                logger.info("too large velocity measurement");
                continue;
            }
            if (InsMath.norm(imu.gyro, 3) > 30.0 * IgnavConstants.D2R) {
                logger.info("too large vehicle turn");
                continue;
            }
            H[IA + nv * nx] = T[i];
            H[IA + 1 + nv * nx] = T[i + 3];
            H[IA + 2 + nv * nx] = T[i + 6];
            H[IV + nv * nx] = C[i];
            H[IV + 1 + nv * nx] = C[i + 3];
            H[IV + 2 + nv * nx] = C[i + 6];

            v[nv] = vb[i];
            r[nv++] = VARVEL_NHC;
        }
        for (i = 0; i < nv; i++)
            R[i + i * nv] = r[i];
        return nv;
    }

    public static int nhc(InsState ins, InsOpt opt, Imud imu) {
        int nx = ins.nx;
        int info = 0;
        int nv;

        double[] H = new double[2 * nx];
        double[] R = new double[2 * 2];
        double[] v = new double[2];
        double[] x = new double[nx];

        nv = bldnhc(opt, imu, ins.Cbe, ins.ve, nx, v, H, R);
        if (nv > 0) {
            info = InsAlignMech.filter(x, ins.P, H, v, R, nx, nv);

            if (info != 0) {
                logger.warn("non-holonomic constraint filter fail");
                info = 0;
            } else {
                ins.stat = IgnavConstants.INSS_NHC;
                info = 1;
                clp(ins, opt, x);
                logger.info("use non-holonomic constraint ok");
            }
        }
        return info;
    }
}