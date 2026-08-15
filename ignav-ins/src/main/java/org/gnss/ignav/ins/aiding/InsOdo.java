package org.gnss.ignav.ins.aiding;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.gnss.ignav.ins.data.Odod;
import org.gnss.ignav.ins.data.Odopt;
import org.gnss.ignav.ins.mech.InsAlignMech;
import org.gnss.ignav.ins.mech.InsMech;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsOdo {

    private static final Logger logger = LoggerFactory.getLogger(InsOdo.class);

    private static final double MAXINOV_ODO = 100.0;
    private static final double VARVEL_ODO = InsMath.SQR(0.05);

    private static double dtAcc = 0.0;
    private static double drAcc = 0.0;

    private InsOdo() {}

    private static void ve2vr(double[] ve, double[] Cbe, double[] Cbr, double[] lever, Imud imu, double s, double[] vr) {
        double[] wl = new double[9], tmp = new double[3], cl = new double[3];
        double[] omgeMat = {0.0, IgnavConstants.OMGE, 0.0, -IgnavConstants.OMGE, 0.0, 0.0, 0.0, 0.0, 0.0};
        InsMath.matmul3v("N", Cbe, lever, cl);
        InsMath.skewsym3(imu.gyro, wl);
        InsMath.matmul3v("N", wl, lever, tmp);
        InsMath.matmul3v("N", Cbe, tmp, vr);
        for (int i = 0; i < 3; i++) vr[i] = ve[i] - vr[i];
        double[] tmp2 = new double[3];
        InsMath.matmul3v("N", omgeMat, cl, tmp2);
        for (int i = 0; i < 3; i++) vr[i] += tmp2[i];
        InsMath.matmul3v("TN", Cbr, vr, tmp);
        InsMath.matcpy(vr, tmp, 1, 3);
        vr[0] *= s;
    }

    private static int odoHVR(InsOpt opt, double[] Cbe, double[] lever, double[] Cbr, double[] ve,
                              double s, Imud imu, Odod odo, int nx, double[] v, double[] H, double[] R) {
        int i, j, nv = 0;
        double[] vr = new double[3];
        double[] r = new double[3];
        double[] ds = new double[3], dbg = new double[9], da = new double[3], dma = new double[3];
        double[] dl = new double[9], ddv = new double[9];
        int ibg = InsStateIdx.xiBg(opt);
        int nbg = InsStateIdx.xnBg(opt);
        int ios = InsStateIdx.xiOs(opt);
        int nos = InsStateIdx.xnOs(opt);
        int iol = InsStateIdx.xiOl(opt);
        int nol = InsStateIdx.xnOl(opt);
        int ioa = InsStateIdx.xiOa(opt);
        int noa = InsStateIdx.xnOa(opt);
        int IV = InsStateIdx.xiV(opt);
        int NV = InsStateIdx.xnV(opt);
        int IA = InsStateIdx.xiA(opt);
        int NA = InsStateIdx.xnA(opt);
        ve2vr(ve, Cbe, Cbr, lever, imu, s, vr);
        ds[0] = vr[0]; ds[1] = 0.0; ds[2] = 0.0;
        double[] wl = new double[9], T = new double[9], cl = new double[3];
        double[] omgeMat = {0.0, IgnavConstants.OMGE, 0.0, -IgnavConstants.OMGE, 0.0, 0.0, 0.0, 0.0, 0.0};
        InsMath.matmul3v("N", Cbe, lever, cl);
        InsMath.skewsym3(ve, wl);
        InsMath.matmul("TN", 3, 3, 3, 1.0, Cbe, wl, 0.0, T);
        InsMath.matmul3v("TN", Cbr, T, da);
        double[] tmp = new double[3];
        InsMath.matmul3v("N", omgeMat, cl, tmp);
        double[] tmp2 = new double[3];
        InsMath.matmul3v("TN", Cbr, tmp, tmp2);
        for (i = 0; i < 3; i++) da[i] += tmp2[i];
        double[] Tdma = new double[9];
        InsMath.matmul3("TN", Cbr, Cbe, Tdma);
        InsMath.matmul3v("N", Tdma, vr, dma);
        double[] wld = new double[9], tmpdbg = new double[9];
        InsMath.skewsym3(lever, wld);
        InsMath.matmul3("NN", Cbe, wld, tmpdbg);
        InsMath.matmul3("TN", Cbr, tmpdbg, dbg);
        for (i = 0; i < 9; i++) dbg[i] = -dbg[i];
        InsMath.matmul3("TN", Cbr, Cbe, ddv);
        double[] wll = new double[9], tmpdl = new double[9], cldl = new double[9];
        InsMath.skewsym3(imu.gyro, wll);
        InsMath.matmul3("NN", Cbe, wll, tmpdl);
        InsMath.matmul3("TN", Cbr, tmpdl, cldl);
        InsMath.matmul("NN", 3, 3, 3, -1.0, cldl, InsMath.eye(3), 0.0, dl);
        InsMath.matmul3("NN", Cbr, omgeMat, tmpdl);
        for (i = 0; i < 9; i++) dl[i] += tmpdl[i];
        for (i = 0; i < (opt.odopt.all != 0 ? 3 : 1); i++) {
            v[nv] = odo.vr[i] - vr[i];
            if (Math.abs(v[nv]) > MAXINOV_ODO) {
                logger.warn("too large innovations for velocity");
            }
            if (H != null) {
                for (j = ios; j < ios + nos; j++) H[j + nv * nx] = ds[i];
                for (j = ioa; j < ioa + noa; j++) H[j + nv * nx] = dma[i + (j - ioa) * 3];
                for (j = iol; j < iol + nol; j++) H[j + nv * nx] = dl[i + (j - iol) * 3];
                for (j = ibg; j < ibg + nbg; j++) H[j + nv * nx] = dbg[i + (j - ibg) * 3];
                for (j = IA; j < IA + NA; j++) H[j + nv * nx] = da[i + (j - IA) * 3];
                for (j = IV; j < IV + NV; j++) H[j + nv * nx] = ddv[i + (j - IV) * 3];
            }
            r[nv] = VARVEL_ODO;
            nv++;
        }
        if (nv > 0) {
            for (i = 0; i < nv; i++) R[i + i * nv] = r[i];
        }
        return nv;
    }

    private static void odoclp(double[] x, InsOpt opt, InsState ins) {
        int ios = InsStateIdx.xiOs(opt);
        int nos = InsStateIdx.xnOs(opt);
        int ioa = InsStateIdx.xiOa(opt);
        int noa = InsStateIdx.xnOa(opt);
        int iol = InsStateIdx.xiOl(opt);
        int nol = InsStateIdx.xnOl(opt);
        if (nos > 0) ins.os -= x[ios];
        if (noa > 0) InsMech.corratt(x, ins.Cbr);
        for (int i = iol; i < iol + nol; i++) ins.rbl[i - iol] += x[i];
        InsNhc.clp(ins, opt, x);
    }

    private static int odofilt(InsOpt opt, Imud imu, Odod odo, InsState ins) {
        int nx = ins.nx, info = 0, nv;
        if (InsMath.norm(imu.gyro, 3) > 30.0 * IgnavConstants.D2R) {
            logger.warn("filter fail due to vehicle have large turn");
            return 0;
        }
        double[] v = new double[3];
        double[] H = new double[3 * nx];
        double[] R = new double[3 * 3];
        double[] x = new double[nx];
        InsMath.matcpy(ins.vr, odo.vr, 1, 3);
        nv = odoHVR(opt, ins.Cbe, ins.lever, ins.Cbr, ins.ve, ins.os, imu, odo, nx, v, H, R);
        if (nv > 0) {
            info = InsAlignMech.filter(x, ins.P, H, v, R, nx, nv);
            if (info != 0) {
                logger.warn("odometry aid ins fail");
                info = 0;
            } else {
                logger.info("odometry aid ins ok");
                ins.stat = IgnavConstants.INSS_ODO;
                info = 1;
                odoclp(x, opt, ins);
            }
        }
        return info;
    }

    private static int odomeas(InsOpt opt, Odod odo, Odod odom) {
        dtAcc += odo.dt;
        drAcc += odo.dr;
        if (dtAcc > (opt.odopt.odt == 0.0 ? 0.5 : opt.odopt.odt)) {
            odom.dt = dtAcc;
            odom.dr = drAcc;
            odom.vr[0] = drAcc / dtAcc;
            dtAcc = 0.0;
            drAcc = 0.0;
            return 1;
        }
        return 0;
    }

    public static int odo(InsOpt opt, Imud imu, Odod odoData, InsState ins) {
        Odod odom = new Odod();
        if (odomeas(opt, odoData, odom) != 0) {
            return odofilt(opt, imu, odom, ins);
        }
        return 0;
    }

    public static void initodo(Odopt odopt, InsState ins) {
        ins.os = odopt.s;
        InsMath.matcpy(ins.rbl, odopt.lever, 3, 1);
    }
}