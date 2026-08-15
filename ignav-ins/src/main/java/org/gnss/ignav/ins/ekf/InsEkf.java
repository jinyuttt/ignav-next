package org.gnss.ignav.ins.ekf;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.gnss.ignav.ins.aiding.InsStateIdx;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsEkf {

    private static final Logger logger = LoggerFactory.getLogger(InsEkf.class);

    private InsEkf() {}

    public static void initEkf(InsState ins, InsOpt opt) {
        int nx = InsStateIdx.nx(opt);
        ins.nx = nx;
        ins.x = new double[nx];
        ins.P = new double[nx * nx];
        ins.P0 = new double[nx * nx];
        ins.F = new double[nx * nx];

        double[] I = InsMath.eye(nx);
        InsMath.matcpy(ins.P, I, nx, nx);
        InsMath.matcpy(ins.P0, I, nx, nx);

        initP(ins, opt);
        InsMath.matcpy(ins.P0, ins.P, nx, nx);
    }

    private static void initP(InsState ins, InsOpt opt) {
        int nx = ins.nx;
        int i;
        double[] P = ins.P;

        for (i = 0; i < nx * nx; i++) P[i] = 0.0;

        int IA = InsStateIdx.xiA(opt);
        int IV = InsStateIdx.xiV(opt);
        int IP = InsStateIdx.xiP(opt);
        int IBG = InsStateIdx.xiBg(opt);
        int IBa = InsStateIdx.xiBa(opt);
        int ISG = InsStateIdx.xiSg(opt);
        int ISA = InsStateIdx.xiSa(opt);
        int IRG = InsStateIdx.xiRg(opt);
        int IRA = InsStateIdx.xiRa(opt);

        for (i = 0; i < 3; i++) P[(IA + i) + (IA + i) * nx] = InsMath.SQR(opt.unc.att);
        for (i = 0; i < 3; i++) P[(IV + i) + (IV + i) * nx] = InsMath.SQR(opt.unc.vel);
        for (i = 0; i < 3; i++) P[(IP + i) + (IP + i) * nx] = InsMath.SQR(opt.unc.pos);

        if (opt.bgopt != 0) {
            for (i = 0; i < 3; i++) P[(IBG + i) + (IBG + i) * nx] = InsMath.SQR(opt.unc.bg);
        }
        if (opt.baopt != 0) {
            for (i = 0; i < 3; i++) P[(IBa + i) + (IBa + i) * nx] = InsMath.SQR(opt.unc.ba);
        }
        if (opt.estsg != 0) {
            for (i = 0; i < 3; i++) P[(ISG + i) + (ISG + i) * nx] = InsMath.SQR(opt.unc.sg);
        }
        if (opt.estsa != 0) {
            for (i = 0; i < 3; i++) P[(ISA + i) + (ISA + i) * nx] = InsMath.SQR(opt.unc.sa);
        }
        if (opt.estrg != 0) {
            for (i = 0; i < 3; i++) P[(IRG + i) + (IRG + i) * nx] = InsMath.SQR(opt.unc.rg);
        }
        if (opt.estra != 0) {
            for (i = 0; i < 3; i++) P[(IRA + i) + (IRA + i) * nx] = InsMath.SQR(opt.unc.ra);
        }
    }

    public static void buildF(InsState ins, InsOpt opt, double dt) {
        int nx = ins.nx;
        double[] F = ins.F;
        double[] Cbe = ins.Cbe;
        double[] re = ins.re;
        double[] ve = ins.ve;
        double[] fb = ins.fb;
        int i;

        for (i = 0; i < nx * nx; i++) F[i] = 0.0;

        int IA = InsStateIdx.xiA(opt);
        int IV = InsStateIdx.xiV(opt);
        int IP = InsStateIdx.xiP(opt);
        int IBG = InsStateIdx.xiBg(opt);
        int IBa = InsStateIdx.xiBa(opt);

        double[] Ceb = new double[9];
        InsMath.matt(Cbe, 3, 3, Ceb);

        double[] fe = new double[3];
        InsMath.matmul3v("N", Cbe, fb, fe);

        double[] Sf = new double[9];
        InsMath.skewsym3(fe, Sf);

        for (i = 0; i < 9; i++) {
            F[(IA) + (IV + i / 3) * nx] += Sf[i] * 0.0;
        }

        double[] Omg = InsMath.OMGE_MAT;
        double[] SOmg = new double[9];
        InsMath.skewsym3(new double[]{Omg[3], Omg[1], 0.0}, SOmg);

        double[] I3 = InsMath.eye(3);
        for (i = 0; i < 3; i++) {
            F[(IV + i) + (IA + i) * nx] = 0.0;
        }

        if (opt.bgopt != 0) {
            double[] negCeb = new double[9];
            for (i = 0; i < 9; i++) negCeb[i] = -Ceb[i];
            for (i = 0; i < 9; i++) {
                F[(IA + i % 3) + (IBG + i / 3) * nx] = negCeb[i];
            }
        }

        if (opt.baopt != 0) {
            for (i = 0; i < 9; i++) {
                F[(IV + i % 3) + (IBa + i / 3) * nx] = -Cbe[i];
            }
        }

        double[] Phi = new double[nx * nx];
        double[] I = InsMath.eye(nx);
        double[] Fdt = new double[nx * nx];
        double[] Fdt2 = new double[nx * nx];

        for (i = 0; i < nx * nx; i++) Fdt[i] = F[i] * dt;
        InsMath.matmul("NN", nx, nx, nx, 1.0, Fdt, Fdt, 0.0, Fdt2);
        for (i = 0; i < 2; i++) Fdt2[i] *= 0.5;

        for (i = 0; i < nx * nx; i++)
            Phi[i] = I[i] + Fdt[i] + 0.5 * Fdt2[i];

        InsMath.matcpy(ins.F, Phi, nx, nx);
    }

    public static void predict(InsState ins, InsOpt opt, double dt) {
        int nx = ins.nx;
        double[] Phi = ins.F;
        double[] Q = new double[nx * nx];
        double[] Pp = new double[nx * nx];
        double[] PhiT = new double[nx * nx];
        int i;

        buildF(ins, opt, dt);

        buildQ(ins, opt, dt, Q);

        InsMath.matt(Phi, nx, nx, PhiT);
        InsMath.matmul("NN", nx, nx, nx, 1.0, Phi, ins.P, 0.0, Pp);
        InsMath.matmul("NN", nx, nx, nx, 1.0, Pp, PhiT, 0.0, ins.P);
        for (i = 0; i < nx * nx; i++) ins.P[i] += Q[i];
    }

    private static void buildQ(InsState ins, InsOpt opt, double dt, double[] Q) {
        int nx = ins.nx;
        int i;

        for (i = 0; i < nx * nx; i++) Q[i] = 0.0;

        int IBG = InsStateIdx.xiBg(opt);
        int IBa = InsStateIdx.xiBa(opt);
        int ISG = InsStateIdx.xiSg(opt);
        int ISA = InsStateIdx.xiSa(opt);

        double psdGyro = opt.psd.gyro;
        double psdAccl = opt.psd.accl;
        double psdBg = opt.psd.bg;
        double psdBa = opt.psd.ba;

        int IA = InsStateIdx.xiA(opt);
        int IV = InsStateIdx.xiV(opt);

        for (i = 0; i < 3; i++) Q[(IA + i) + (IA + i) * nx] = psdGyro * dt;
        for (i = 0; i < 3; i++) Q[(IV + i) + (IV + i) * nx] = psdAccl * dt;

        if (opt.bgopt != 0) {
            for (i = 0; i < 3; i++) Q[(IBG + i) + (IBG + i) * nx] = psdBg * dt;
        }
        if (opt.baopt != 0) {
            for (i = 0; i < 3; i++) Q[(IBa + i) + (IBa + i) * nx] = psdBa * dt;
        }
    }
}