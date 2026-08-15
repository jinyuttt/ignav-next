package org.gnss.ignav.ins.mech;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.GTime;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsAlign;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsAlignMech {

    private static final Logger logger = LoggerFactory.getLogger(InsAlignMech.class);

    private static final double MIN_ACCL = 0.05;
    private static final double MIN_GRYO = 0.05 * IgnavConstants.D2R;
    private static final double VAR_VEL = InsMath.SQR(1.0);
    private static final double VAR_MVEL = InsMath.SQR(0.1);
    private static final double EPS = 1E-2;
    private static final double EPSX = 1E-3;

    private InsAlignMech() {}

    public static boolean chkstatic(Imud data, InsOpt opt, double gn) {
        return (data.accl[0] < MIN_ACCL) && (data.accl[1] < MIN_ACCL)
                && (Math.abs(data.accl[2] + gn) < MIN_ACCL)
                && (data.gyro[0] < MIN_GRYO) && (data.gyro[1] < MIN_GRYO) && (data.gyro[2] < MIN_GRYO);
    }

    public static void normdcm(double[] C) {
        double[] Cp = new double[9];
        double[] T = InsMath.eye(3);
        double[] CC = InsMath.eye(3);
        InsMath.matcpy(Cp, C, 3, 3);
        InsMath.matmul("TN", 3, 3, 3, 1.0, Cp, Cp, -1.0, CC);
        for (int i = 0; i < 9; i++)
            T[i] = T[i] - 0.5 * CC[i];
        InsMath.matmul("NN", 3, 3, 3, 1.0, T, Cp, 0.0, C);
    }

    public static int coarseAlign(InsState ins, Imud[] data, int n, InsOpt opt) {
        int i, j, k;
        double[] pos = new double[3];
        double[] fb = new double[3], omg = new double[3];
        double[] gn = new double[3];
        double g;
        double[] A = new double[9], B = new double[9], Cne = new double[9];
        double[] fbc = new double[3], omgc = new double[3];

        if (n <= 0) return 0;

        InsMath.ecef2pos(ins.re, pos);
        InsMath.ned2xyz(pos, Cne);
        InsMath.gravityNed(pos, gn);
        g = InsMath.norm(gn, 3);

        for (k = 0, i = 0; i < n; i++) {
            if (opt.align.chkstatic != 0) {
                if (!chkstatic(data[i], opt, g)) continue;
            }
            k++;
            for (j = 0; j < 3; j++) {
                fb[j] += data[i].accl[j];
                omg[j] += data[i].gyro[j];
            }
        }
        if (k <= 0) {
            logger.warn("no enough static imu data");
            return 0;
        }
        for (i = 0; i < 3; i++) {
            fb[i] /= k;
            omg[i] /= k;
        }

        InsMech.insErrmodel(fb, omg, fbc, omgc, ins);

        A[0] = -Math.tan(pos[0]) / g;
        A[2] = -1.0 / g;
        A[3] = 1.0 / (IgnavConstants.OMGE * Math.cos(pos[0]));
        A[7] = -1.0 / (g * IgnavConstants.OMGE * Math.cos(pos[0]));

        InsMath.cross3(fbc, omgc, gn);
        for (i = 0; i < 3; i++) {
            B[0 + i * 3] = fbc[i];
            B[1 + i * 3] = omgc[i];
        }
        for (i = 0; i < 3; i++) {
            B[2 + i * 3] = gn[i];
        }

        InsMath.matmul("NN", 3, 3, 3, 1.0, A, B, 0.0, ins.Cbn);
        normdcm(ins.Cbn);

        InsMath.matmul("NN", 3, 3, 3, 1.0, Cne, ins.Cbn, 1.0, ins.Cbe);
        normdcm(ins.Cbe);

        ins.time = new GTime(data[k].time);
        return 1;
    }

    public static void asiBlkMat(double[] A, int m, int n, double[] B, int p, int q, int isr, int isc) {
        for (int i = isr; i < isr + p; i++) {
            for (int j = isc; j < isc + q; j++) {
                A[i + j * m] = B[(i - isr) + (j - isc) * p];
            }
        }
    }

    public static void imuConInc(Imud[] data, int offset, int ns, double dt, double[] phim, double[] dvbm) {
        int i, j;
        for (i = 0; i < 3; i++)
            phim[i] = dvbm[i] = 0.0;
        for (i = 0; i < ns; i++) {
            for (j = 0; j < 3; j++) {
                phim[j] += data[offset + i].gyro[j] * dt;
                dvbm[j] += data[offset + i].accl[j] * dt;
            }
        }
    }

    public static void a2qua(double[] att, double[] q) {
        double[] att2 = new double[3];
        double sp, sr, sy, cp, cr, cy;
        for (int i = 0; i < 3; i++)
            att2[i] = att[i] / 2.0;
        sp = Math.sin(att2[0]); sr = Math.sin(att2[1]); sy = Math.sin(att2[2]);
        cp = Math.cos(att2[0]); cr = Math.cos(att2[1]); cy = Math.cos(att2[2]);
        q[0] = cp * cr * cy - sp * sr * sy;
        q[1] = sp * cr * cy - cp * sr * sy;
        q[2] = cp * sr * cy + sp * cr * sy;
        q[3] = cp * cr * sy + sp * sr * cy;
    }

    public static void rv2m(double[] rv, double[] m) {
        double xx, yy, zz, n2, a, b, n;
        xx = rv[0] * rv[0]; yy = rv[1] * rv[1]; zz = rv[2] * rv[2];
        n2 = xx + yy + zz;
        if (n2 < 1E-8) {
            a = 1.0 - n2 * (1.0 / 6.0 - n2 / 120.0);
            b = 0.5 - n2 * (1.0 / 24.0 - n2 / 720.0);
        } else {
            n = Math.sqrt(n2);
            a = Math.sin(n) / n;
            b = (1.0 - Math.cos(n)) / n2;
        }
        double arvx = a * rv[0], arvy = a * rv[1], arvz = a * rv[2];
        double bxx = b * xx, bxy = b * rv[0] * rv[1], bxz = b * rv[0] * rv[2];
        double byy = b * yy, byz = b * rv[1] * rv[2], bzz = b * zz;
        m[0] = 1 - byy - bzz; m[3] = -arvz + bxy; m[6] = arvy + bxz;
        m[1] = arvz + bxy;    m[4] = 1 - bxx - bzz; m[7] = -arvx + byz;
        m[2] = -arvy + bxz;   m[5] = arvx + byz;    m[8] = 1 - bxx - byy;
    }

    public static void qconj(double[] qi, double[] qo) {
        qo[0] = qi[0]; qo[1] = -qi[1]; qo[2] = -qi[2]; qo[3] = -qi[3];
    }

    public static void qmulve(double[] q, double[] vi, double[] vo) {
        double qo1, qo2, qo3, qo4;
        qo1 = -q[1] * vi[0] - q[2] * vi[1] - q[3] * vi[2];
        qo2 = q[0] * vi[0] + q[2] * vi[2] - q[3] * vi[1];
        qo3 = q[0] * vi[1] + q[3] * vi[0] - q[1] * vi[2];
        qo4 = q[0] * vi[2] + q[1] * vi[1] - q[2] * vi[0];
        vo[0] = -qo1 * q[1] + qo2 * q[0] - qo3 * q[3] + qo4 * q[2];
        vo[1] = -qo1 * q[2] + qo3 * q[0] - qo4 * q[1] + qo2 * q[3];
        vo[2] = -qo1 * q[3] + qo4 * q[0] - qo2 * q[2] + qo3 * q[1];
    }

    public static void qmul(double[] q1, double[] q2, double[] q) {
        q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
        q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
        q[2] = q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3];
        q[3] = q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1];
    }

    public static void qupdt(double[] rv, double[] q) {
        double n2, c, s, n, n_2;
        double[] q2 = new double[4], q1 = new double[4];
        InsMath.matcpy(q1, q, 1, 4);
        n2 = rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2];
        if (n2 < 1E-8) {
            c = 1.0 - n2 * (1.0 / 8.0 - n2 / 384.0);
            s = 1.0 / 2.0 - n2 * (1.0 / 48.0 - n2 / 3840.0);
        } else {
            n = Math.sqrt(n2);
            n_2 = n / 2.0;
            c = Math.cos(n_2);
            s = Math.sin(n_2) / n;
        }
        q2[0] = c;
        for (int i = 0; i < 3; i++) q2[i + 1] = s * rv[i];
        q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
        q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
        q[2] = q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3];
        q[3] = q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1];
        double nq2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        if (nq2 > 1.000001 || nq2 < 0.99999) {
            double sq = Math.sqrt(nq2);
            for (int i = 0; i < 4; i++) q[i] = q[i] / sq;
        }
    }

    public static void qdelphi(double[] qpb, double[] phi) {
        double[] q = new double[4], qpbc = new double[4];
        InsMath.matcpy(qpbc, qpb, 1, 4);
        InsMath.rov2qua(phi, q);
        qmul(q, qpbc, qpb);
    }

    public static void q2att(double[] q, double[] rpy) {
        double q00 = q[0] * q[0], q11 = q[1] * q[1], q22 = q[2] * q[2], q33 = q[3] * q[3];
        rpy[0] = Math.atan2(2.0 * (q[2] * q[3] + q[0] * q[1]), q00 - q11 - q22 + q33);
        rpy[1] = Math.asin(-2.0 * (q[1] * q[3] - q[0] * q[2]));
        rpy[2] = Math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q00 + q11 - q22 - q33);
    }

    public static void q2mat(double[] q, double[] C) {
        double q00 = q[0] * q[0], q11 = q[1] * q[1], q22 = q[2] * q[2], q33 = q[3] * q[3];
        C[0] = q00 + q11 - q22 - q33;
        C[1] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
        C[2] = 2.0 * (q[1] * q[3] + q[0] * q[2]);
        C[3] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
        C[4] = q00 - q11 + q22 - q33;
        C[5] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
        C[6] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
        C[7] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
        C[8] = q00 - q11 - q22 + q33;
    }

    public static int filter(double[] x, double[] P, double[] H, double[] v,
                             double[] R, int nx, int nv) {
        double[] Hp = new double[nv * nx];
        double[] K = new double[nx * nv];
        double[] S = new double[nv * nv];
        double[] V = new double[nv];
        double[] T = new double[nx * nx];
        double[] Pk = new double[nx * nx];
        int i, j;

        InsMath.matmul("TN", nv, nv, nx, 1.0, H, P, 0.0, S);
        InsMath.matmul("NN", nv, nv, nv, 1.0, S, R, 1.0, S);

        for (i = 0; i < nv; i++) {
            if (Math.abs(S[i * nv + i]) < 1E-20) return -1;
        }

        double[] Sinv = new double[nv * nv];
        InsMath.matcpy(Sinv, S, nv, nv);
        if (InsMath.matinv(Sinv, nv) != 0) return -1;

        InsMath.matmul("NN", nx, nv, nx, 1.0, P, H, 0.0, Hp);
        InsMath.matmul("TN", nx, nv, nv, 1.0, Hp, Sinv, 0.0, K);

        InsMath.matmul("NN", nv, 1, nx, 1.0, H, x, 0.0, V);
        for (i = 0; i < nv; i++) V[i] = v[i] - V[i];

        for (i = 0; i < nx; i++) {
            double s = 0.0;
            for (j = 0; j < nv; j++) s += K[i * nv + j] * V[j];
            x[i] += s;
        }

        double[] KH = new double[nx * nx];
        InsMath.matmul("NN", nx, nx, nv, 1.0, K, H, 0.0, KH);
        double[] I = InsMath.eye(nx);
        for (i = 0; i < nx * nx; i++) T[i] = I[i] - KH[i];
        InsMath.matmul("NN", nx, nx, nx, 1.0, T, P, 0.0, Pk);
        for (i = 0; i < nx; i++) {
            for (j = 0; j < nx; j++) {
                P[i * nx + j] = 0.5 * (Pk[i * nx + j] + Pk[j * nx + i]);
            }
        }
        return 0;
    }

    private static int afnfilter(double[] x, double[] P, double[] Q, double[] R,
                                double[] H, double[] fn, double[] Phi, int nx, int nv) {
        double[] x_ = new double[nx];
        double[] P_ = new double[nx * nx];
        double[] v = new double[nv];
        int i;

        InsMath.matmul("NN", nx, 1, nx, 1.0, Phi, x, 0.0, x_);
        double[] PhiT = new double[nx * nx];
        InsMath.matt(Phi, nx, nx, PhiT);
        double[] tmp = new double[nx * nx];
        InsMath.matmul("NN", nx, nx, nx, 1.0, Phi, P, 0.0, tmp);
        InsMath.matmul("NN", nx, nx, nx, 1.0, tmp, PhiT, 0.0, P_);
        for (i = 0; i < nx * nx; i++) P_[i] += Q[i];

        InsMath.matcpy(v, fn, nv, 1);
        InsMath.matmul("TN", nv, 1, nx, -1.0, H, x_, 1.0, v);

        if (filter(x_, P_, H, v, R, nx, nv) != 0) return 0;
        InsMath.matcpy(x, x_, nx, 1);
        InsMath.matcpy(P, P_, nx, nx);
        return 1;
    }

    public static int fineAlign(InsState ins, Imud[] data, int n, InsOpt opt) {
        int k1 = 1, k2 = 1;
        double[] rpyfn = new double[3], rpyvn = new double[3];
        InsAlign pas = opt.align;

        if (n <= 0 || pas.dt <= 0.0) {
            logger.warn("ins initial align fail");
            return 0;
        }

        double[] qfn = new double[4], qvn = new double[4];
        if (opt.alignFn != 0) {
            k1 = alignfn(ins, data, n, pas.phi0, opt, pas.dt, rpyfn, qfn);
        }
        if (opt.alignVn != 0) {
            k2 = alignvn(ins, data, n, pas.phi0, pas.wvn, opt, pas.dt, qvn, rpyvn);
        }

        if (k1 != 0 && k2 != 0) {
            if (opt.alignFn != 0 && opt.alignVn != 0) {
                if (arequatclose(qvn, qfn)) {
                    double[] qavg = new double[4];
                    avgquat(qavg, qfn, qvn);
                    InsMath.quat2dcm(qavg, ins.Cbn);
                    ins.time = new GTime(data[Math.min(k1, k2)].time);
                } else {
                    logger.warn("fn-alignment and vn-alignment is not coincidence");
                    return 0;
                }
            } else if (opt.alignVn != 0) {
                InsMath.quat2dcm(qvn, ins.Cbn);
            } else if (opt.alignFn != 0) {
                InsMath.quat2dcm(qfn, ins.Cbn);
            } else {
                logger.warn("ins initial align failed,because ins initial options don't set");
                return 0;
            }
        } else {
            logger.warn("ins initial align failed");
            return 0;
        }
        return 1;
    }

    private static int alignfn(InsState ins, Imud[] data, int n, double[] phi0,
                               InsOpt opt, double dt, double[] rpyo, double[] qo) {
        int i, j, k, ns;
        double[] phim = new double[3], dvbm = new double[3];
        double[] Cnn = new double[9], gn = new double[3], wie = new double[3];
        double[] tv = new double[3], fn = new double[3], t = new double[3], rpy = new double[3], xb = new double[5];
        double[] P, Q, R, H, x, Phi;
        double[] q = new double[4], qinv = new double[4];

        InsMath.gravityNed(ins.rn, gn);
        ns = opt.align.ns == 0 ? 1 : opt.align.ns;

        x = new double[5]; P = new double[25]; Q = new double[25];
        R = new double[4]; H = new double[10]; Phi = new double[25];

        afnkfinit(dt, ins.rn, phi0, opt, Phi, Q, R, P, H, x);

        wie[0] = IgnavConstants.OMGE * Math.cos(ins.rn[0]);
        wie[1] = 0.0;
        wie[2] = -IgnavConstants.OMGE * Math.sin(ins.rn[0]);

        for (i = 0; i < 3; i++) tv[i] = -wie[i] * ns * dt / 2.0;
        InsMath.rov2dcm(tv, Cnn);

        InsMath.dcm2quat(ins.Cbn, q);

        for (k = 0, i = 0; i < n; i += ns) {
            if (opt.align.chkstatic != 0) {
                if (!chkstatic(data[i], opt, gn[2])) continue;
            }
            imuConInc(data, i, ns, dt, phim, dvbm);
            qconj(q, qinv);
            qmulve(qinv, wie, tv);
            for (j = 0; j < 3; j++) phim[j] -= tv[j] * dt * ns;
            qupdt(phim, q);
            for (j = 0; j < 3; j++) tv[j] = dvbm[j] / (ns * dt);
            qmulve(q, tv, t);
            InsMath.matmul("NN", 3, 1, 3, 1.0, Cnn, t, 0.0, fn);
            if (afnfilter(x, P, Q, R, H, fn, Phi, 5, 2) == 0) continue;
            InsMath.matcpy(xb, x, 1, 3);
            for (j = 0; j < 2; j++) x[j] = -x[j];
            qdelphi(q, x);
            for (j = 0; j < 3; j++) x[j] = 1E-15;
            k++;
            q2att(q, rpy);
        }

        if (InsMath.norm(xb, 3) < EPSX) {
            InsMath.matcpy(qo, q, 1, 4);
            InsMath.matcpy(rpyo, rpy, 1, 3);
        } else {
            logger.warn("ins initial align use kalman filter with fn as measurement fail");
            return 0;
        }
        return k;
    }

    private static void afnkfinit(double dt, double[] pos, double[] phi0, InsOpt opt,
                                  double[] phi, double[] Q, double[] R, double[] P0, double[] H, double[] x0) {
        double[] we = new double[3], W = new double[9];
        double[] II = InsMath.eye(5);
        double[] gn = new double[3];
        InsAlign pa = opt.align;
        int i;

        for (i = 0; i < 3; i++) Q[i + i * 5] = InsMath.SQR(pa.web[i]) * dt;
        for (i = 0; i < 2; i++) R[i + i * 2] = InsMath.SQR(pa.wdb[i] / Math.sqrt(dt));
        for (i = 0; i < 3; i++) P0[i + i * 5] = InsMath.SQR(phi0[i]);
        for (i = 3; i < 5; i++) P0[i + i * 5] = InsMath.SQR(pa.eb[i - 3]);
        for (i = 0; i < 5; i++) x0[i] = 1E-20;

        we[0] = IgnavConstants.OMGE * Math.cos(pos[0]);
        we[1] = 0.0;
        we[2] = -IgnavConstants.OMGE * Math.sin(pos[0]);
        InsMath.skewsym3(we, W);
        asiBlkMat(phi, 5, 5, W, 3, 3, 0, 0);
        double[] I24 = {-1, 0, 0, -1};
        asiBlkMat(phi, 5, 5, I24, 2, 2, 0, 3);

        InsMath.gravityNed(pos, gn);
        H[5] = gn[2]; H[1] = -gn[2];

        for (i = 0; i < 25; i++) phi[i] = II[i] + phi[i] * dt;
    }

    private static int alignvn(InsState ins, Imud[] data, int n, double[] phi0,
                               double[] wvn, InsOpt opt, double dt, double[] qo, double[] rpyo) {
        int i, j, k, ns;
        double[] Cnn = new double[9], tv = new double[3], we = new double[3];
        double[] phim = new double[3], dvbm = new double[3], rpy = new double[3], xb = new double[12];
        double[] gn = new double[3], Cbn = new double[9], Ct = new double[9], Ct1 = new double[9], Wv = new double[9];
        double[] dvn = new double[3], vn = new double[3], wn = new double[3];
        double[] x, P, H, Q, R, Phi;
        double[] q = new double[4];

        InsMath.gravityNed(ins.rn, gn);
        ns = opt.align.ns == 0 ? 1 : opt.align.ns;

        x = new double[12]; P = new double[144]; Q = new double[144];
        R = new double[9]; H = new double[36]; Phi = InsMath.eye(12);

        avnkfinit(dt, ins.rn, phi0, opt, wvn, Phi, Q, R, P, H, x);

        we[0] = IgnavConstants.OMGE * Math.cos(ins.rn[0]);
        we[1] = 0.0;
        we[2] = -IgnavConstants.OMGE * Math.sin(ins.rn[0]);
        for (i = 0; i < 3; i++) tv[i] = -we[i] * ns * dt / 2.0;
        InsMath.rov2dcm(tv, Cnn);

        wnin(ins.rn, ins.vn, wn);

        InsMath.dcm2quat(ins.Cbn, q);
        InsMath.matcpy(Cbn, ins.Cbn, 3, 3);
        InsMath.matcpy(vn, ins.vn, 1, 3);

        for (k = 0, i = 0; i < n; i++) {
            if (opt.align.chkstatic != 0) {
                if (!chkstatic(data[i], opt, gn[2])) continue;
            }
            imuConInc(data, i, ns, dt, phim, dvbm);
            InsMath.quat2dcm(q, Cbn);
            InsMath.matmul33("NNN", Cnn, Cbn, dvbm, 3, 3, 3, 1, dvn);
            for (j = 0; j < 3; j++) vn[j] += (dvn[j] + gn[j] * ns * dt);
            InsMath.matmul("TN", 3, 1, 3, -dt * ns, Cbn, wn, 1.0, phim);
            qupdt(phim, q);
            for (j = 0; j < 9; j++) Ct[j] = -Cbn[j] * ns * dt;
            for (j = 0; j < 9; j++) Ct1[j] = -Ct[j];
            InsMath.skewsym3(dvn, Wv);
            asiBlkMat(Phi, 12, 12, Wv, 3, 3, 3, 0);
            asiBlkMat(Phi, 12, 12, Ct, 3, 3, 0, 6);
            asiBlkMat(Phi, 12, 12, Ct1, 3, 3, 3, 9);
            if (afnfilter(x, P, Q, R, H, vn, Phi, 12, 3) == 0) continue;
            InsMath.matcpy(xb, x, 1, 12);
            for (j = 0; j < 2; j++) x[j] = -x[j];
            qdelphi(q, x);
            for (j = 0; j < 3; j++) x[j] = 1E-15;
            for (j = 3; j < 6; j++) { vn[j - 3] -= x[j]; x[j] = 1E-15; }
            wnin(ins.rn, vn, wn);
            k++;
            q2att(q, rpy);
        }

        if (InsMath.norm(xb, 6) < EPSX) {
            InsMath.matcpy(qo, q, 1, 4);
            InsMath.matcpy(rpyo, rpy, 3, 1);
        } else {
            logger.warn("ins initial align use kalman filter with vn as measurement fail");
            return 0;
        }
        return k;
    }

    private static void avnkfinit(double dt, double[] pos, double[] phi0, InsOpt opt,
                                  double[] wvn, double[] Phi, double[] Q, double[] R, double[] P, double[] H, double[] x) {
        InsAlign pa = opt.align;
        double[] we = new double[3], W = new double[9], gn = new double[3];
        double[] I3 = InsMath.eye(3);
        int i;

        for (i = 0; i < 3; i++) Q[i + i * 12] = InsMath.SQR(pa.web[i]) * dt;
        for (i = 3; i < 6; i++) Q[i + i * 12] = InsMath.SQR(pa.wdb[i - 3]) * dt;
        for (i = 0; i < 3; i++) R[i + i * 3] = InsMath.SQR(wvn[i]);
        for (i = 0; i < 3; i++) P[i + i * 12] = InsMath.SQR(phi0[i]);
        for (i = 3; i < 6; i++) P[i + i * 12] = VAR_VEL;
        for (i = 6; i < 9; i++) P[i + i * 12] = InsMath.SQR(pa.eb[i - 6]);
        for (i = 9; i < 12; i++) P[i + i * 12] = InsMath.SQR(pa.db[i - 9]);
        for (i = 0; i < 12; i++) x[i] = 1E-15;

        we[0] = IgnavConstants.OMGE * Math.cos(pos[0]);
        we[1] = 0.0;
        we[2] = -IgnavConstants.OMGE * Math.sin(pos[0]);
        InsMath.skewsym3(we, W);
        asiBlkMat(Phi, 12, 12, W, 3, 3, 0, 0);
        asiBlkMat(Phi, 12, 12, I3, 3, 3, 3, 0);
        asiBlkMat(Phi, 12, 12, I3, 3, 3, 6, 0);

        InsMath.gravityNed(pos, gn);
        double[] Fv = new double[9];
        Fv[2] = gn[2]; Fv[5] = gn[2];
        asiBlkMat(H, 3, 12, Fv, 3, 2, 0, 0);
    }

    private static void wnin(double[] rn, double[] vn, double[] wn) {
        double[] rN = new double[1], rE = new double[1];
        InsMech.radii(rn, rN, rE);
        double N = rN[0], M = rE[0];
        wn[0] = vn[1] / (N + rn[2]);
        wn[1] = -vn[0] / (M + rn[2]);
        wn[2] = -IgnavConstants.OMGE * Math.sin(rn[0]) - vn[1] * Math.tan(rn[0]) / (N + rn[2]);
    }

    private static boolean arequatclose(double[] q1, double[] q2) {
        double[] dq = new double[4], qinv = new double[4];
        qconj(q1, qinv);
        qmul(qinv, q2, dq);
        return (Math.abs(1.0 - dq[0]) < EPS) && (Math.abs(InsMath.norm(dq, 3)) < EPS);
    }

    private static void avgquat(double[] qavg, double[] q1, double[] q2) {
        InsMath.quatnormalize(q1);
        InsMath.quatnormalize(q2);
        for (int i = 0; i < 4; i++) qavg[i] = 0.5 * (q1[i] + q2[i]);
    }
}