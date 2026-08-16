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

        double[] fe = new double[3];
        InsMath.matmul3v("N", Cbe, fb, fe);

        double[] negSf = new double[9];
        InsMath.skewsym3(fe, negSf);
        for (i = 0; i < 9; i++) negSf[i] = -negSf[i];
        asiBlkMat(F, nx, nx, negSf, 3, 3, IA, IV);

        double[] I3 = InsMath.eye(3);
        asiBlkMat(F, nx, nx, I3, 3, 3, IV, IP);

        if (opt.bgopt != 0) {
            double[] Ceb = new double[9];
            InsMath.matt(Cbe, 3, 3, Ceb);
            for (i = 0; i < 9; i++) Ceb[i] = -Ceb[i];
            asiBlkMat(F, nx, nx, Ceb, 3, 3, IBG, IA);
        }

        if (opt.baopt != 0) {
            double[] negCbe = new double[9];
            for (i = 0; i < 9; i++) negCbe[i] = -Cbe[i];
            asiBlkMat(F, nx, nx, negCbe, 3, 3, IBa, IV);
        }

        double[] Phi = new double[nx * nx];
        double[] I = InsMath.eye(nx);
        double[] Fdt = new double[nx * nx];
        double[] Fdt2 = new double[nx * nx];

        for (i = 0; i < nx * nx; i++) Fdt[i] = F[i] * dt;
        InsMath.matmul("NN", nx, nx, nx, 1.0, Fdt, Fdt, 0.0, Fdt2);
        for (i = 0; i < nx * nx; i++) Fdt2[i] *= 0.5;

        for (i = 0; i < nx * nx; i++)
            Phi[i] = I[i] + Fdt[i] + 0.5 * Fdt2[i];

        InsMath.matcpy(ins.F, Phi, nx, nx);
    }

    private static void asiBlkMat(double[] A, int ra, int ca, double[] B, int rb, int cb, int rowOff, int colOff) {
        for (int i = 0; i < rb; i++) {
            for (int j = 0; j < cb; j++) {
                A[(rowOff + i) + (colOff + j) * ra] += B[i + j * rb];
            }
        }
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

        int IA = InsStateIdx.xiA(opt);
        int IV = InsStateIdx.xiV(opt);
        int IP = InsStateIdx.xiP(opt);
        int IBG = InsStateIdx.xiBg(opt);
        int IBa = InsStateIdx.xiBa(opt);
        int ISG = InsStateIdx.xiSg(opt);
        int ISA = InsStateIdx.xiSa(opt);
        int IRG = InsStateIdx.xiRg(opt);
        int IRA = InsStateIdx.xiRa(opt);
        int IDT = InsStateIdx.xiDt(opt);
        int ILA = InsStateIdx.xiLever(opt);
        int IOS = InsStateIdx.xiOs(opt);
        int IOL = InsStateIdx.xiOl(opt);
        int IOA = InsStateIdx.xiOa(opt);

        sysQ(IA, 3, nx, opt.psd.gyro, dt, Q);
        sysQ(IV, 3, nx, opt.psd.accl, dt, Q);
        if (opt.baopt != 0) sysQ(IBa, 3, nx, opt.psd.ba, dt, Q);
        if (opt.bgopt != 0) sysQ(IBG, 3, nx, opt.psd.bg, dt, Q);
        if (opt.estdt != 0) sysQ(IDT, InsStateIdx.xnDt(opt), nx, opt.psd.dt, dt, Q);
        if (opt.estsg != 0) sysQ(ISG, InsStateIdx.xnSg(opt), nx, opt.psd.sg, dt, Q);
        if (opt.estsa != 0) sysQ(ISA, InsStateIdx.xnSa(opt), nx, opt.psd.sa, dt, Q);
        if (opt.estrg != 0) sysQ(IRG, InsStateIdx.xnRg(opt), nx, opt.psd.rg, dt, Q);
        if (opt.estra != 0) sysQ(IRA, InsStateIdx.xnRa(opt), nx, opt.psd.ra, dt, Q);
        if (opt.estlever != 0) sysQ(ILA, InsStateIdx.xnLever(opt), nx, opt.psd.ol, dt, Q);
        if (opt.estodos != 0) sysQ(IOS, InsStateIdx.xnOs(opt), nx, opt.psd.os, dt, Q);
        if (opt.estodol != 0) sysQ(IOL, InsStateIdx.xnOl(opt), nx, opt.psd.ol, dt, Q);
        if (opt.estodoa != 0) sysQ(IOA, InsStateIdx.xnOa(opt), nx, opt.psd.oa, dt, Q);
    }

    private static void sysQ(int is, int n, int nx, double v, double dt, double[] Q) {
        for (int i = is; i < is + n; i++)
            Q[i + i * nx] = v * Math.abs(dt);
    }

    public static void precPhi(InsState ins, InsOpt opt, double dt) {
        int nx = ins.nx;
        double[] F = new double[nx * nx];
        InsMath.matcpy(F, ins.F, nx, nx);
        for (int i = 0; i < nx * nx; i++)
            F[i] *= dt;
        expmat(F, nx, ins.F);
    }

    public static void getPhi1(InsState ins, InsOpt opt, double dt) {
        int nx = ins.nx;
        double[] F = new double[nx * nx];
        double[] I = InsMath.eye(nx);
        InsMath.matcpy(F, ins.F, nx, nx);
        for (int i = 0; i < nx * nx; i++)
            F[i] *= dt;
        for (int i = 0; i < nx * nx; i++)
            ins.F[i] = I[i] + F[i];
    }

    private static void expmat(double[] A, int n, double[] E) {
        double[] I = InsMath.eye(n);
        double[] S = new double[n * n];
        double[] T = new double[n * n];
        InsMath.matcpy(S, A, n, n);
        InsMath.matcpy(E, I, n, n);
        for (int k = 1; k <= 12; k++) {
            double scale = 1.0 / k;
            InsMath.matmul("NN", n, n, n, scale, A, S, 0.0, T);
            double[] tmp = S;
            S = T;
            T = tmp;
            for (int i = 0; i < n * n; i++)
                E[i] += S[i];
        }
    }
}