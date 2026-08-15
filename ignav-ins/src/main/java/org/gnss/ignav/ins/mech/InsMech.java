package org.gnss.ignav.ins.mech;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.common.Quaternion;
import org.gnss.ignav.ins.data.GTime;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsMech {

    private static final Logger logger = LoggerFactory.getLogger(InsMech.class);

    public static final int INSS_MECH = 1;

    public static final int IMUCOOR_FRD = 0;
    public static final int IMUCOOR_RFU = 1;

    public static final int IMUDECFMT_RATE = 0;
    public static final int IMUDECFMT_INCR = 1;

    public static final int IMUVALFMT_RAD = 0;
    public static final int IMUVALFMT_DEG = 1;

    private InsMech() {}

    public static void insErrmodel(double[] accl, double[] gyro, double[] corAccl, double[] corGyro, InsState ins) {
        double[] Ma = ins.Ma;
        double[] Mg = ins.Mg;
        double[] ba = ins.ba;
        double[] bg = ins.bg;
        double[] Gg = ins.Gg;
        int i, j;
        double[] Mai = new double[9], Mgi = new double[9];
        double[] I = InsMath.eye(3);
        double[] T = new double[6];
        double[] Gf = new double[3];

        for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
                Mai[i * 3 + j] = I[i * 3 + j] + Ma[i * 3 + j];
        for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
                Mgi[i * 3 + j] = I[i * 3 + j] + Mg[i * 3 + j];

        if (InsMath.matinv(Mai, 3) == 0 && InsMath.matinv(Mgi, 3) == 0) {
            InsMath.matmul("NN", 3, 1, 3, 1.0, Mai, accl, 0.0, T);
            InsMath.matmul("NN", 3, 1, 3, 1.0, Mgi, gyro, 0.0, T);
        }
        if (corAccl != null) {
            for (i = 0; i < 3; i++)
                corAccl[i] = T[i] - ba[i];
            InsMath.matmul("NN", 3, 1, 3, 1.0, Gg, accl, 0.0, Gf);
        }
        if (corGyro != null) {
            for (i = 0; i < 3; i++)
                corGyro[i] = T[3 + i] - bg[i] - Gf[i];
        }
    }

    public static void insErrmodel2(double[] accl, double[] gyro, double[] Ma, double[] Mg,
                                    double[] ba, double[] bg, double[] Gg,
                                    double[] corAccl, double[] corGyro) {
        int i, j;
        double[] Mai = new double[9], Mgi = new double[9];
        double[] I = InsMath.eye(3);
        double[] T = new double[6];
        double[] Gf = new double[3];

        for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
                Mai[i * 3 + j] = I[i * 3 + j] + Ma[i * 3 + j];
        for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
                Mgi[i * 3 + j] = I[i * 3 + j] + Mg[i * 3 + j];

        if (InsMath.matinv(Mai, 3) == 0 && InsMath.matinv(Mgi, 3) == 0) {
            InsMath.matmul("NN", 3, 1, 3, 1.0, Mai, accl, 0.0, T);
            InsMath.matmul("NN", 3, 1, 3, 1.0, Mgi, gyro, 0.0, T);
            System.arraycopy(T, 0, T, 3, 3);
        }
        if (corAccl != null) {
            for (i = 0; i < 3; i++)
                corAccl[i] = T[i] - ba[i];
            InsMath.matmul("NN", 3, 1, 3, 1.0, Gg, accl, 0.0, Gf);
        }
        if (corGyro != null) {
            for (i = 0; i < 3; i++)
                corGyro[i] = T[3 + i] - bg[i] - Gf[i];
        }
    }

    public static void qmulv(double[] vi, Quaternion quat, double[] vo) {
        double[] C = new double[9];
        Quaternion.toRhRotMatrix(quat, C);
        InsMath.matmul("NN", 3, 1, 3, 1.0, C, vi, 0.0, vo);
    }

    public static void quatupd(double[] vi, Quaternion qo) {
        double[] dq = new double[4];
        InsMath.rov2qua(vi, dq);
        Quaternion q1 = new Quaternion(dq[0], dq[1], dq[2], dq[3]);
        Quaternion q2 = Quaternion.copy(qo);
        Quaternion.mul(qo, q1, q2);
    }

    public static void initins(InsState ins, double[] re, double angh, Imud[] data, int n, InsOpt opt) {
        double[] pos = new double[3];
        double[] rpy = new double[3];
        double[] Cnb = new double[9], Cne = new double[9];
        double[] fb = new double[3], ab = new double[3];
        double[] ge = new double[3], gb = new double[3];
        int i, j;

        logger.debug("initins: n={} angh={}", n, angh * IgnavConstants.R2D);

        InsMath.ecef2pos(re, pos);
        rpy[2] = angh;

        for (i = 0; i < 3; i++) {
            ins.rn[i] = pos[i];
            ins.vn[i] = ins.an[i] = ins.ba[i] = ins.bg[i] = ins.fb[i] = 0.0;
        }
        for (i = 0; i < 3; i++) {
            ins.re[i] = re[i];
            ins.ve[i] = ins.ae[i] = ins.ba[i] = ins.bg[i] = ins.fb[i] = 0.0;
        }
        if (n > 0) {
            for (i = 0; i < 3; i++) {
                for (j = 0; j < n; j++) {
                    fb[i] += data[j].accl[i];
                    ab[i] += data[j].gyro[i];
                }
                fb[i] /= n;
                ab[i] /= n;
            }
            rpy[0] = Math.atan2(-fb[1], -fb[2]);
            rpy[1] = Math.atan(fb[0] / InsMath.norm(fb, 2));

            if (angh == 0.0) {
                double[] headArr = new double[1];
                InsMath.rp2head(rpy[0], rpy[1], ab, headArr);
                rpy[2] = headArr[0];
            }
            ins.time = new GTime(data[n - 1].time);
        }
        InsMath.rpy2dcm(rpy, Cnb);
        InsMath.matt(Cnb, 3, 3, ins.Cbn);
        InsMath.ned2xyz(pos, Cne);
        InsMath.matmul3("NT", Cne, Cnb, ins.Cbe);
        if (n > 0) {
            InsMath.gravity(re, ge);
            InsMath.matmul3v("T", ins.Cbe, ge, gb);
            for (i = 0; i < 3; i++) {
                ins.ba[i] = fb[i] + gb[i];
                ins.bg[i] = ab[i];
            }
        }
    }

    public static void savepins(InsState ins, Imud data) {
        System.arraycopy(ins.omgb, 0, ins.omgbp, 0, 3);
        System.arraycopy(ins.fb, 0, ins.fbp, 0, 3);
        System.arraycopy(ins.re, 0, ins.pins, 0, 3);
        System.arraycopy(ins.ve, 0, ins.pins, 3, 3);
        System.arraycopy(ins.Cbe, 0, ins.pCbe, 0, 9);
    }

    public static void updateatt(double t, double[] Cbe, double[] omgb, double[] das) {
        double[] alpha = new double[3];
        double a, a1, a2;
        double[] Ca = new double[9], Ca2 = new double[9], Comg = new double[9], Cbep = new double[9];
        double[] Cbb = InsMath.eye(3);
        double[] Cei = new double[9];
        int i;

        for (i = 0; i < 3; i++)
            alpha[i] = omgb[i] * t + das[i];

        InsMath.skewsym3(alpha, Ca);
        InsMath.matmul3("NN", Ca, Ca, Ca2);
        a = InsMath.norm(alpha, 3);
        if (a < 1E-8) {
            a1 = 1.0 - a * a / 6.0;
            a2 = 0.5 - a * a / 24.0;
        } else {
            a1 = Math.sin(a) / a;
            a2 = (1.0 - Math.cos(a)) / (a * a);
        }
        for (i = 0; i < 9; i++)
            Cbb[i] += a1 * Ca[i] + a2 * Ca2[i];

        if (InsMath.INSUPDPRE == 1) {
            Cei[0] = Math.cos(IgnavConstants.OMGE * t);
            Cei[3] = Math.sin(IgnavConstants.OMGE * t);
            Cei[1] = -Math.sin(IgnavConstants.OMGE * t);
            Cei[4] = Math.cos(IgnavConstants.OMGE * t);
            Cei[8] = 1.0;
            InsMath.matmul3("NN", Cei, Cbe, Cbep);
            InsMath.matmul3("NN", Cbep, Cbb, Cbe);
        } else {
            InsMath.matmul3("NN", Cbe, Cbb, Cbep);
            InsMath.matmul3("NN", InsMath.OMGE_MAT, Cbe, Comg);
            for (i = 0; i < 9; i++)
                Cbe[i] = Cbep[i] - Comg[i] * t;
        }
    }

    public static void updinsn(InsState ins) {
        double[] Cne = new double[9];

        InsMath.ecef2pos(ins.re, ins.rn);
        InsMath.ned2xyz(ins.rn, Cne);
        InsMath.matmul("TN", 3, 3, 3, 1.0, Cne, ins.Cbe, 0.0, ins.Cbn);
        InsMath.matmul("TN", 3, 1, 3, 1.0, Cne, ins.ve, 0.0, ins.vn);
        InsMath.matmul("TN", 3, 1, 3, 1.0, Cne, ins.ae, 0.0, ins.an);
    }

    public static void updateInsStateN(InsState ins) {
        double[] Cne = new double[9];
        InsMath.ecef2pos(ins.re, ins.rn);
        InsMath.ned2xyz(ins.rn, Cne);
        InsMath.matmul("TN", 3, 3, 3, 1.0, Cne, ins.Cbe, 0.0, ins.Cbn);
        InsMath.matmul("TN", 3, 1, 3, 1.0, Cne, ins.ve, 0.0, ins.vn);
        InsMath.matmul("TN", 3, 1, 3, 1.0, Cne, ins.ae, 0.0, ins.an);
    }

    public static void updateInsStateE(InsState ins) {
        double[] Cne = new double[9];
        InsMath.ned2xyz(ins.rn, Cne);
        InsMath.matmul("NN", 3, 3, 3, 1.0, Cne, ins.Cbn, 0.0, ins.Cbe);
        InsMath.matmul("NN", 3, 1, 3, 1.0, Cne, ins.vn, 0.0, ins.ve);
        InsMath.pos2ecef(ins.rn, ins.re);
        InsMath.getaccl(ins.fb, ins.Cbe, ins.re, ins.ve, ins.ae);
    }

    public static void geoparam(double[] pos, double[] vn, double[] wenN, double[] wieN,
                                double[] g, double[] ReOut, double[] RnOut) {
        double sr, Re, Rn, sL2, sL4;

        sr = Math.sqrt(1.0 - IgnavConstants.E_SQR * InsMath.SQR(Math.sin(pos[0])));
        Re = (IgnavConstants.RE_WGS84 / sr) + pos[2];
        Rn = (IgnavConstants.RE_WGS84 * (1.0 - IgnavConstants.E_SQR) / (sr * sr * sr)) + pos[2];

        if (ReOut != null) ReOut[0] = Re;
        if (RnOut != null) RnOut[0] = Rn;

        if (wenN != null) {
            wenN[0] = vn[1] / Re;
            wenN[1] = -vn[0] / Rn;
            wenN[2] = -vn[1] * Math.tan(pos[0]) / Re;
        }
        if (wieN != null) {
            wieN[0] = IgnavConstants.OMGE * Math.cos(pos[0]);
            wieN[1] = 0.0;
            wieN[2] = -IgnavConstants.OMGE * Math.sin(pos[0]);
        }
        if (g != null) {
            sL2 = InsMath.SQR(Math.sin(pos[0]));
            sL4 = InsMath.SQR(sL2);
            double a1 = 9.7803267715;
            double a2 = 0.0052790414;
            double a3 = 0.0000232718;
            double a4 = -0.0000030876910891;
            double a5 = 0.0000000043977311;
            double a6 = 0.0000000000007211;
            g[0] = a1 * (1 + a2 * sL2 + a3 * sL4) + (a4 + a5 * sL2) * pos[2] + a6 * InsMath.SQR(pos[2]);
        }
    }

    public static void radii(double[] rn, double[] rN, double[] rE) {
        rN[0] = IgnavConstants.RE_WGS84 * (1.0 - InsMath.SQR(IgnavConstants.WGS_E))
                / Math.pow(1.0 - InsMath.SQR(IgnavConstants.WGS_E) * InsMath.SQR(Math.sin(rn[0])), 1.5);
        rE[0] = IgnavConstants.RE_WGS84
                / Math.sqrt(1.0 - InsMath.SQR(IgnavConstants.WGS_E) * InsMath.SQR(Math.sin(rn[0])));
    }

    public static void getqbn(InsState ins, double[] qbn) {
        double[] pos = new double[3];
        double[] Cne = new double[9], Cbn = new double[9];
        InsMath.ecef2pos(ins.re, pos);
        InsMath.ned2xyz(pos, Cne);
        InsMath.matmul3("TN", Cne, ins.Cbe, Cbn);
        InsMath.dcm2quat(Cbn, qbn);
    }

    public static void rotscullCorr(InsState ins, InsOpt opt, double dt, double[] dv, double[] da) {
        double[] dap = new double[3], dvp = new double[3];
        double[] dak = new double[3], dvk = new double[3];
        double[] dv1 = new double[3], dv2 = new double[3], dv3 = new double[3];
        double domg, a1, a2;
        double[] dv4 = new double[3];
        int i;

        for (i = 0; i < 3; i++) {
            dap[i] = ins.omgbp[i] * ins.dt;
            dvp[i] = ins.fbp[i] * ins.dt;
        }
        for (i = 0; i < 3; i++) {
            dak[i] = ins.omgb[i] * dt;
            dvk[i] = ins.fb[i] * dt;
        }
        InsMath.cross3(dak, dvk, dv1);
        InsMath.cross3(dap, dvk, dv2);
        InsMath.cross3(dvp, dak, dv3);

        domg = InsMath.norm(dak, 3);
        if (Math.abs(domg) > 1E-6) {
            a1 = (1.0 - Math.cos(domg)) / InsMath.SQR(domg);
            a2 = 1.0 / InsMath.SQR(domg) * (1.0 - Math.sin(domg) / domg);
        } else {
            a1 = 0.5 - InsMath.SQR(domg) / 24.0 + InsMath.SQR(InsMath.SQR(domg)) / 720.0;
            a2 = 1.0 / 6.0 - InsMath.SQR(domg) / 120.0 + InsMath.SQR(InsMath.SQR(domg)) / 5040.0;
        }
        InsMath.cross3(dak, dv1, dv4);

        if (dv != null) {
            for (i = 0; i < 3; i++) {
                dv[i] = a1 * dv1[i] + a2 * dv4[i] + 1.0 / 12.0 * (dv2[i] + dv3[i]);
            }
        }
        if (da != null) {
            InsMath.cross3(dap, dak, da);
            for (i = 0; i < 3; i++)
                da[i] = 1.0 / 12.0 * da[i];
        }
    }

    public static void rmlever(double[] ve, double[] Cbe, double[] Omg, double[] lever, double[] vec) {
        double[] T = new double[9], TT = new double[3], wl = new double[3];
        InsMath.skewsym3(Omg, T);
        InsMath.matmul("NN", 3, 1, 3, 1.0, T, lever, 0.0, wl);
        InsMath.matmul3v("N", Cbe, wl, TT);
        InsMath.matmul33("NNN", InsMath.OMGE_MAT, Cbe, lever, 3, 3, 3, 1, wl);
        if (vec != null) {
            for (int i = 0; i < 3; i++)
                vec[i] = ve[i] + TT[i] - wl[i];
        }
    }

    public static void gapv2ipv(double[] pos, double[] vel, double[] Cbe, double[] lever,
                                Imud imu, double[] posi, double[] veli) {
        int i;
        double[] T = new double[9], TT = new double[3];

        if (posi != null) {
            System.arraycopy(pos, 0, posi, 0, 3);
            InsMath.matmul("NN", 3, 1, 3, -1.0, Cbe, lever, 1.0, posi);
        }
        if (veli != null) {
            InsMath.skewsym3(imu.gyro, T);
            InsMath.matmul("NN", 3, 1, 3, 1.0, T, lever, 0.0, TT);
            InsMath.matmul("NN", 3, 1, 3, 1.0, Cbe, TT, 0.0, T);
            InsMath.matmul33("NNN", InsMath.OMGE_MAT, Cbe, lever, 3, 3, 3, 1, TT);
            for (i = 0; i < 3; i++)
                veli[i] = vel[i] - T[i] + TT[i];
        }
    }

    public static void corratt(double[] dx, double[] C) {
        double[] T = new double[9];
        double[] I = InsMath.eye(3);
        InsMath.skewsym3(dx, T);
        for (int i = 0; i < 9; i++)
            I[i] -= T[i];
        InsMath.matcpy(T, C, 3, 3);
        InsMath.matmul("NN", 3, 3, 3, 1.0, I, T, 0.0, C);
    }

    public static void getatt(InsState ins, double[] rpy) {
        double[] llh = new double[3], C = new double[9], Cnb = new double[9];
        InsMath.ecef2pos(ins.re, llh);
        InsMath.ned2xyz(llh, C);
        InsMath.matmul("TN", 3, 3, 3, 1.0, ins.Cbe, C, 0.0, Cnb);
        InsMath.dcm2rpy(Cnb, rpy);
    }

    public static void adjustimu(InsOpt opt, Imud imu) {
        double[] gyro = new double[3], accl = new double[3];
        double dt = 1.0 / opt.hz;
        int i;

        if (opt.imucoors == IMUCOOR_RFU) {
            System.arraycopy(imu.gyro, 0, gyro, 0, 3);
            System.arraycopy(imu.accl, 0, accl, 0, 3);
            InsMath.matmul("NN", 3, 1, 3, 1.0, InsMath.CRF, gyro, 0.0, imu.gyro);
            InsMath.matmul("NN", 3, 1, 3, 1.0, InsMath.CRF, accl, 0.0, imu.accl);
        }
        if (opt.imudecfmt == IMUDECFMT_INCR) {
            for (i = 0; i < 3; i++) {
                imu.gyro[i] /= dt;
                imu.accl[i] /= dt;
            }
        }
        if (opt.imuvalfmt == IMUVALFMT_DEG) {
            for (i = 0; i < 3; i++)
                imu.gyro[i] *= IgnavConstants.D2R;
        }
    }

    public static int updateins(InsOpt insopt, InsState ins, Imud data) {
        double dt;
        double[] Cbe = new double[9];
        double[] fe = new double[3], ge = new double[3], cori = new double[3];
        double[] Cbb = InsMath.eye(3);
        double[] Ca = new double[9], Ca2 = new double[9];
        double a1, a2, a;
        double[] alpha = new double[3];
        double[] Omg = new double[9];
        double[] ae = new double[3];
        double[] das = new double[3], dvs = new double[3];
        double[] fb = new double[3];
        int i;

        savepins(ins, data);

        dt = GTime.timeDiff(data.time, ins.time);
        if (dt > InsMath.MAXDT || Math.abs(dt) < 1E-6) {
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
                insErrmodel(data.accl, data.gyro, ins.fb, ins.omgb, ins);
            } else {
                ins.omgb[i] = data.gyro[i] - ins.bg[i];
                ins.fb[i] = data.accl[i] - ins.ba[i];
            }
        }

        if (InsMath.SCULL_CORR == 1) {
            rotscullCorr(ins, insopt, dt, dvs, das);
        }

        updateatt(dt, ins.Cbe, ins.omgb, das);

        if (InsMath.INSUPDPRE == 1) {
            for (i = 0; i < 3; i++) alpha[i] = ins.omgb[i] * dt + das[i];
            InsMath.skewsym3(alpha, Ca);
            a = InsMath.norm(alpha, 3);
            if (a > 1E-8) {
                a1 = (1.0 - Math.cos(a)) / InsMath.SQR(a);
                a2 = 1.0 / InsMath.SQR(a) * (1.0 - Math.sin(a) / a);
                InsMath.matmul3("NN", Ca, Ca, Ca2);
                for (i = 0; i < 9; i++) Cbb[i] += a1 * Ca[i] + a2 * Ca2[i];
                InsMath.skewsym3(ae, Omg);
                InsMath.matmul3("NN", Cbe, Cbb, Ca);
                InsMath.matmul3("NN", Omg, Cbe, Ca2);
                for (i = 0; i < 9; i++) ins.Cbe[i] = Ca[i] - 0.5 * Ca2[i];
            } else {
                InsMath.skewsym3(ae, Omg);
                InsMath.matmul3("NN", Omg, Cbe, Ca);
                for (i = 0; i < 9; i++) ins.Cbe[i] -= 0.5 * Ca[i];
            }
        } else {
            for (i = 0; i < 9; i++) ins.Cbe[i] = (Cbe[i] + ins.Cbe[i]) / 2.0;
        }

        for (i = 0; i < 3; i++) fb[i] = ins.fb[i] + dvs[i] / dt;
        InsMath.matmul3v("N", ins.Cbe, fb, fe);

        if (insopt.gravityex != 0) {
            InsMath.pregrav(ins.re, ge);
        } else {
            InsMath.gravity(ins.re, ge);
        }

        InsMath.matmul3v("N", InsMath.OMGE_MAT, ins.ve, cori);
        for (i = 0; i < 3; i++) {
            ins.ae[i] = fe[i] + ge[i] - 2.0 * cori[i];
            ins.ve[i] += ins.ae[i] * dt;
            ins.re[i] += ins.ve[i] * dt + ins.ae[i] / 2.0 * dt * dt;
        }

        updateInsStateN(ins);

        ins.dt = GTime.timeDiff(data.time, ins.time);
        ins.ptime = new GTime(ins.time);
        ins.time = new GTime(data.time);
        ins.stat = INSS_MECH;

        return 1;
    }

    public static int updateinsn(InsOpt insopt, InsState ins, Imud data) {
        double dt;
        double[] vmid = new double[3], wenN = new double[3], wieN = new double[3];
        double[] rN = new double[1], rE = new double[1], gVal = new double[1];
        double[] dv = new double[3], dv1 = new double[3], dv2 = new double[3];
        double[] qbn = new double[4], w = new double[3], rn = new double[3];
        double[] da = new double[3], qb = new double[4], qn = new double[4], q = new double[4];
        double[] das = new double[3], dvs = new double[3];
        int i;

        updateInsStateN(ins);
        savepins(ins, data);

        dt = GTime.timeDiff(data.time, ins.time);
        if (dt > InsMath.MAXDT || Math.abs(dt) < 1E-6) {
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
                insErrmodel(data.accl, data.gyro, ins.fb, ins.omgb, ins);
            } else {
                ins.omgb[i] = data.gyro[i] - ins.bg[i];
                ins.fb[i] = data.accl[i] - ins.ba[i];
            }
        }
        System.arraycopy(ins.vn, 0, vmid, 0, 3);
        System.arraycopy(ins.rn, 0, rn, 0, 3);

        geoparam(rn, vmid, wenN, wieN, gVal, rE, rN);

        if (InsMath.SCULL_CORR == 1) {
            rotscullCorr(ins, insopt, dt, dvs, das);
        }

        for (i = 0; i < 3; i++)
            dv[i] = ins.fb[i] * dt + dvs[i];

        getqbn(ins, qbn);
        InsMath.quatrot(qbn, dv, 0, dv1);

        for (i = 0; i < 3; i++)
            w[i] = 2.0 * wieN[i] + wenN[i];
        InsMath.cross3(ins.vn, w, dv2);
        dv2[2] = gVal[0] + dv2[2];

        for (i = 0; i < 3; i++)
            dv2[i] = dv2[i] * dt;
        for (i = 0; i < 3; i++) {
            ins.vn[i] = vmid[i] + dv1[i] + dv2[i];
            ins.an[i] = (dv1[i] + dv2[i]) / dt;
        }

        for (i = 0; i < 3; i++)
            vmid[i] = (vmid[i] + ins.vn[i]) / 2.0;
        for (i = 0; i < 3; i++)
            da[i] = ins.omgb[i] * dt + das[i];
        InsMath.rvec2quat(da, qb);
        InsMath.quatmul(qbn, qb, q);

        geoparam(rn, vmid, wenN, wieN, gVal, rE, rN);
        for (i = 0; i < 3; i++)
            da[i] = -(wenN[i] + wieN[i]) * dt;
        InsMath.rvec2quat(da, qn);
        InsMath.quatmul(qn, q, qbn);
        InsMath.quat2dcm(qbn, ins.Cbn);

        ins.rn[0] += 1.0 / rN[0] * vmid[0] * dt;
        ins.rn[1] += 1.0 / rE[0] / Math.cos(rn[0]) * vmid[1] * dt;
        ins.rn[2] -= vmid[2] * dt;

        updateInsStateE(ins);

        ins.dt = dt;
        ins.ptime = new GTime(ins.time);
        ins.time = new GTime(data.time);
        ins.stat = INSS_MECH;

        return 1;
    }

    public static void estatt(Imud[] data, int n, double[] Cbn) {
        double[] fb = new double[3], ab = new double[3], rpy = new double[3];
        double[] Cnb = new double[9];
        int i, j;

        if (n > 0) {
            for (i = 0; i < 3; i++) {
                for (j = 0; j < n; j++) {
                    fb[i] += data[j].accl[i];
                    ab[i] += data[j].gyro[i];
                }
                fb[i] /= n;
                ab[i] /= n;
            }
            rpy[0] = Math.atan2(-fb[1], -fb[2]);
            rpy[1] = Math.atan(fb[0] / InsMath.norm(fb, 2));
            double[] headArr = new double[1];
            InsMath.rp2head(rpy[0], rpy[1], ab, headArr);
            rpy[2] = headArr[0];
        }
        InsMath.rpy2dcm(rpy, Cnb);
        InsMath.matt(Cnb, 3, 3, Cbn);
    }
}