package org.gnss.ignav.ins.common;

public final class InsMath {

    private InsMath() {}

    public static final double MAXDT = 60.0;
    public static final int INSUPDPRE = 1;
    public static final int SCULL_CORR = 1;

    public static final double[] OMGE_MAT = {
        0.0, IgnavConstants.OMGE, 0.0,
        -IgnavConstants.OMGE, 0.0, 0.0,
        0.0, 0.0, 0.0
    };

    public static final double[] CRF = {
        0.0, 1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, -1.0
    };

    public static void matmul(String tr, int n, int k, int m, double alpha,
                              double[] A, double[] B, double beta, double[] C) {
        int tra = tr.charAt(0) == 'T' ? 1 : 0;
        int trb = tr.length() > 1 && tr.charAt(1) == 'T' ? 1 : 0;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < k; j++) {
                double s = 0.0;
                for (int p = 0; p < m; p++) {
                    int ai = tra == 1 ? p * n + i : i * m + p;
                    int bi = trb == 1 ? j * m + p : p * k + j;
                    s += A[ai] * B[bi];
                }
                C[i * k + j] = alpha * s + beta * C[i * k + j];
            }
        }
    }

    public static void matmul3(String tr, double[] A, double[] B, double[] C) {
        matmul(tr, 3, 3, 3, 1.0, A, B, 0.0, C);
    }

    public static void matmul3v(String tr, double[] A, double[] b, double[] c) {
        int tA = tr.charAt(0) == 'T' ? 1 : 0;
        for (int i = 0; i < 3; i++) {
            double s = 0.0;
            for (int j = 0; j < 3; j++) {
                int ai = tA == 1 ? j * 3 + i : i * 3 + j;
                s += A[ai] * b[j];
            }
            c[i] = s;
        }
    }

    public static void matmul33(String tr, double[] A, double[] B, double[] C,
                                int n, int m, int p, int q, double[] D) {
        double[] T = new double[n * p];
        matmul(tr.substring(0, 2), n, p, m, 1.0, A, B, 0.0, T);
        matmul("N" + tr.substring(2), n, q, p, 1.0, T, C, 0.0, D);
    }

    public static void skewsym3(double[] ang, double[] C) {
        C[0] = 0.0;      C[1] = -ang[2];   C[2] = ang[1];
        C[3] = ang[2];   C[4] = 0.0;       C[5] = -ang[0];
        C[6] = -ang[1];  C[7] = ang[0];    C[8] = 0.0;
    }

    public static void skewsym3x(double x, double y, double z, double[] C) {
        C[0] = 0.0;  C[1] = -z;   C[2] = y;
        C[3] = z;    C[4] = 0.0;  C[5] = -x;
        C[6] = -y;   C[7] = x;    C[8] = 0.0;
    }

    public static void setzero(double[] A, int n, int m) {
        for (int i = 0; i < n * m; i++) A[i] = 0.0;
    }

    public static void seteye(double[] A, int n) {
        setzero(A, n, n);
        for (int i = 0; i < n; i++) A[i * n + i] = 1.0;
    }

    public static double[] eye(int n) {
        double[] A = new double[n * n];
        seteye(A, n);
        return A;
    }

    public static void matcpy(double[] dst, double[] src, int n, int m) {
        System.arraycopy(src, 0, dst, 0, n * m);
    }

    public static void matt(double[] src, int n, int m, double[] dst) {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                dst[j * n + i] = src[i * m + j];
            }
        }
    }

    public static double norm(double[] a, int n) {
        double s = 0.0;
        for (int i = 0; i < n; i++) s += a[i] * a[i];
        return Math.sqrt(s);
    }

    public static void cross3(double[] a, double[] b, double[] c) {
        c[0] = a[1] * b[2] - a[2] * b[1];
        c[1] = a[2] * b[0] - a[0] * b[2];
        c[2] = a[0] * b[1] - a[1] * b[0];
    }

    public static double dot(double[] a, double[] b, int n) {
        double s = 0.0;
        for (int i = 0; i < n; i++) s += a[i] * b[i];
        return s;
    }

    public static double SQR(double a) {
        return a * a;
    }

    public static int matinv(double[] A, int n) {
        int stride = 2 * n;
        double[] B = new double[n * stride];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                B[i * stride + j] = A[i * n + j];
                B[i * stride + n + j] = (i == j) ? 1.0 : 0.0;
            }
        }

        for (int i = 0; i < n; i++) {
            int maxRow = i;
            double maxVal = Math.abs(B[i * stride + i]);
            for (int k = i + 1; k < n; k++) {
                if (Math.abs(B[k * stride + i]) > maxVal) {
                    maxVal = Math.abs(B[k * stride + i]);
                    maxRow = k;
                }
            }
            if (maxVal < 1e-20) return -1;

            if (maxRow != i) {
                for (int j = 0; j < stride; j++) {
                    double tmp = B[i * stride + j];
                    B[i * stride + j] = B[maxRow * stride + j];
                    B[maxRow * stride + j] = tmp;
                }
            }

            double piv = B[i * stride + i];
            for (int j = 0; j < stride; j++) {
                B[i * stride + j] /= piv;
            }

            for (int k = 0; k < n; k++) {
                if (k == i) continue;
                double factor = B[k * stride + i];
                for (int j = 0; j < stride; j++) {
                    B[k * stride + j] -= factor * B[i * stride + j];
                }
            }
        }

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                A[i * n + j] = B[i * stride + n + j];
            }
        }
        return 0;
    }

    public static void ned2xyz(double[] pos, double[] Cne) {
        double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);
        Cne[0] = -sinp * cosl;  Cne[1] = -sinl;       Cne[2] = -cosp * cosl;
        Cne[3] = -sinp * sinl;  Cne[4] = cosl;         Cne[5] = -cosp * sinl;
        Cne[6] = cosp;          Cne[7] = 0.0;           Cne[8] = -sinp;
    }

    public static void rpy2dcm(double[] rpy, double[] Cnb) {
        double sinPhi = Math.sin(rpy[0]), cosPhi = Math.cos(rpy[0]);
        double sinTheta = Math.sin(rpy[1]), cosTheta = Math.cos(rpy[1]);
        double sinPsi = Math.sin(rpy[2]), cosPsi = Math.cos(rpy[2]);
        Cnb[0] = cosTheta * cosPsi;
        Cnb[1] = cosTheta * sinPsi;
        Cnb[2] = -sinTheta;
        Cnb[3] = -cosPhi * sinPsi + sinPhi * sinTheta * cosPsi;
        Cnb[4] = cosPhi * cosPsi + sinPhi * sinTheta * sinPsi;
        Cnb[5] = sinPhi * cosTheta;
        Cnb[6] = sinPhi * sinPsi + cosPhi * sinTheta * cosPsi;
        Cnb[7] = -sinPhi * cosPsi + cosPhi * sinTheta * sinPsi;
        Cnb[8] = cosPhi * cosTheta;
    }

    public static void dcm2quat(double[] C, double[] q) {
        double[][] a = new double[3][3];
        double s, tr;
        a[0][0] = C[0]; a[0][1] = C[1]; a[0][2] = C[2];
        a[1][0] = C[3]; a[1][1] = C[4]; a[1][2] = C[5];
        a[2][0] = C[6]; a[2][1] = C[7]; a[2][2] = C[8];

        tr = a[0][0] + a[1][1] + a[2][2];
        if (tr > 0) {
            s = 0.5 / Math.sqrt(tr + 1.0);
            q[0] = 0.25 / s;
            q[1] = (a[2][1] - a[1][2]) * s;
            q[2] = (a[0][2] - a[2][0]) * s;
            q[3] = (a[1][0] - a[0][1]) * s;
        } else {
            if (a[0][0] > a[1][1] && a[0][0] > a[2][2]) {
                s = 2.0 * Math.sqrt(1.0 + a[0][0] - a[1][1] - a[2][2]);
                q[0] = (a[2][1] - a[1][2]) / s;
                q[1] = 0.25 * s;
                q[2] = (a[0][1] + a[1][0]) / s;
                q[3] = (a[0][2] + a[2][0]) / s;
            } else if (a[1][1] > a[2][2]) {
                s = 2.0 * Math.sqrt(1.0 + a[1][1] - a[0][0] - a[2][2]);
                q[0] = (a[0][2] - a[2][0]) / s;
                q[1] = (a[0][1] + a[1][0]) / s;
                q[2] = 0.25 * s;
                q[3] = (a[1][2] + a[2][1]) / s;
            } else {
                s = 2.0 * Math.sqrt(1.0 + a[2][2] - a[0][0] - a[1][1]);
                q[0] = (a[1][0] - a[0][1]) / s;
                q[1] = (a[0][2] + a[2][0]) / s;
                q[2] = (a[1][2] + a[2][1]) / s;
                q[3] = 0.25 * s;
            }
        }
    }

    public static void quat2dcm(double[] q, double[] C) {
        double sqw = q[0] * q[0], sqx = q[1] * q[1];
        double sqy = q[2] * q[2], sqz = q[3] * q[3];
        double invs = 1.0 / (sqx + sqy + sqz + sqw);
        double tmp1, tmp2;

        C[0] = (sqx - sqy - sqz + sqw) * invs;
        C[4] = (-sqx + sqy - sqz + sqw) * invs;
        C[8] = (-sqx - sqy + sqz + sqw) * invs;

        tmp1 = q[1] * q[2]; tmp2 = q[3] * q[0];
        C[3] = 2.0 * (tmp1 + tmp2) * invs;
        C[1] = 2.0 * (tmp1 - tmp2) * invs;

        tmp1 = q[1] * q[3]; tmp2 = q[2] * q[0];
        C[6] = 2.0 * (tmp1 + tmp2) * invs;
        C[2] = 2.0 * (tmp1 - tmp2) * invs;

        tmp1 = q[2] * q[3]; tmp2 = q[1] * q[0];
        C[7] = 2.0 * (tmp1 + tmp2) * invs;
        C[5] = 2.0 * (tmp1 - tmp2) * invs;
    }

    public static void dcm2rpy(double[] Cnb, double[] rpy) {
        rpy[0] = Math.atan2(Cnb[5], Cnb[8]);
        rpy[1] = -Math.asin(Cnb[2]);
        rpy[2] = Math.atan2(Cnb[1], Cnb[0]);
    }

    public static void rov2dcm(double[] rv, double[] C) {
        double a, a1, a2;
        double[] Ca = new double[9], Ca2 = new double[9];
        skewsym3(rv, Ca);
        matmul3("NN", Ca, Ca, Ca2);
        a = norm(rv, 3);
        if (a < 1E-8) {
            a1 = 1.0 - a * a / 6.0;
            a2 = 0.5 - a * a / 24.0;
        } else {
            a1 = Math.sin(a) / a;
            a2 = (1.0 - Math.cos(a)) / (a * a);
        }
        seteye(C, 3);
        for (int i = 0; i < 9; i++) {
            C[i] += a1 * Ca[i] + a2 * Ca2[i];
        }
    }

    public static void rov2qua(double[] rv, double[] q) {
        double a = norm(rv, 3);
        double s;
        if (a < 1E-8) {
            q[0] = 1.0;
            s = 0.5;
        } else {
            q[0] = Math.cos(a / 2.0);
            s = Math.sin(a / 2.0) / a;
        }
        q[1] = s * rv[0];
        q[2] = s * rv[1];
        q[3] = s * rv[2];
    }

    public static void quat2rv(double[] q, double[] rv) {
        double n = Math.sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        double angle = 2.0 * Math.atan2(n, q[0]);
        if (n < 1E-8) {
            rv[0] = 2.0 * q[1] / q[0];
            rv[1] = 2.0 * q[2] / q[0];
            rv[2] = 2.0 * q[3] / q[0];
            return;
        }
        double s = angle / n;
        rv[0] = s * q[1];
        rv[1] = s * q[2];
        rv[2] = s * q[3];
    }

    public static void rvec2quat(double[] rvec, double[] q) {
        double rotAng = Math.sqrt(SQR(rvec[0]) + SQR(rvec[1]) + SQR(rvec[2]));
        if (Math.abs(rotAng) < 1E-8) {
            q[0] = 1.0;
            q[1] = q[2] = q[3] = 0.0;
        } else {
            double cR = Math.cos(rotAng / 2.0);
            double sR = Math.sin(rotAng / 2.0);
            q[0] = cR;
            q[1] = rvec[0] / rotAng * sR;
            q[2] = rvec[1] / rotAng * sR;
            q[3] = rvec[2] / rotAng * sR;
        }
    }

    public static void dcm2rot(double[] C, double[] rv) {
        double sinPHI = 0.5 * Math.sqrt(SQR(C[5] - C[7]) + SQR(C[6] - C[2]) + SQR(C[1] - C[3]));
        double cosPHI = 0.5 * (C[0] + C[4] + C[8] - 1.0);
        double PHI = Math.atan2(sinPHI, cosPHI);
        double u1, u2, u3;

        if (cosPHI >= 0) {
            if (sinPHI == 0) {
                rv[0] = rv[1] = rv[2] = 0.0;
                return;
            }
            u1 = (C[5] - C[7]) / (2.0 * sinPHI);
            u2 = (C[6] - C[2]) / (2.0 * sinPHI);
            u3 = (C[1] - C[3]) / (2.0 * sinPHI);
        } else {
            double sr_a = Math.sqrt(1.0 + 2.0 * C[0] - SQR(C[0] + C[4] + C[8] - 1.0)) / 2.0;
            if (sr_a > 1E-8) {
                u1 = sr_a;
                u2 = (C[3] + C[1]) / (4.0 * sr_a);
                u3 = (C[6] + C[2]) / (4.0 * sr_a);
            } else {
                sr_a = Math.sqrt(1.0 + 2.0 * C[4] - SQR(C[0] + C[4] + C[8] - 1.0)) / 2.0;
                u1 = (C[3] + C[1]) / (4.0 * sr_a);
                u2 = sr_a;
                u3 = (C[7] + C[5]) / (4.0 * sr_a);
            }
        }
        rv[0] = PHI * u1;
        rv[1] = PHI * u2;
        rv[2] = PHI * u3;
    }

    public static void enu2ned(double[] enu, double[] ned, double[] C) {
        if (enu != null && ned != null) {
            ned[0] = enu[1];
            ned[1] = enu[0];
            ned[2] = -enu[2];
        }
        if (C != null) {
            setzero(C, 3, 3);
            C[1] = C[3] = 1.0;
            C[8] = -1.0;
        }
    }

    public static void rfu2frd(double[] rfu, double[] frd, double[] C) {
        if (rfu != null && frd != null) {
            frd[0] = rfu[1];
            frd[1] = rfu[0];
            frd[2] = -rfu[2];
        }
        if (C != null) {
            setzero(C, 3, 3);
            C[1] = C[3] = 1.0;
            C[8] = -1.0;
        }
    }

    public static void gravityNed(double[] pos, double[] gn) {
        double g0 = 9.7803253359 * (1.0 + 0.001931853 * SQR(Math.sin(pos[0])))
                / Math.sqrt(1.0 - SQR(IgnavConstants.WGS_E * Math.sin(pos[0])));
        gn[0] = -8.08E-9 * pos[2] * Math.sin(2.0 * pos[0]);
        gn[1] = 0.0;
        gn[2] = g0 * (1.0 - 2.0 / IgnavConstants.RE_WGS84
                * (1.0 + IgnavConstants.FE_WGS84 * (1.0 - 2 * SQR(Math.sin(pos[0])))
                + SQR(IgnavConstants.OMGE * IgnavConstants.RE_WGS84) * IgnavConstants.RP / IgnavConstants.MU)
                * pos[2] + 3.0 / SQR(IgnavConstants.RE_WGS84) * SQR(pos[2]));
    }

    public static double gravity0(double[] pos) {
        double e2 = IgnavConstants.FE_WGS84 * (2.0 - IgnavConstants.FE_WGS84);
        double sinp = Math.sin(pos[0]);
        return 9.7803253359 * (1.0 + 0.001931853 * sinp * sinp)
                / Math.sqrt(1 - e2 * sinp * sinp);
    }

    public static void gravity(double[] re, double[] ge) {
        double[] pos = new double[3];
        double[] gn = new double[3];
        double[] Cne = new double[9];
        ecef2pos(re, pos);
        gn[2] = gravity0(pos) * (1.0 - 2.0 * pos[2] / IgnavConstants.RE_WGS84);
        ned2xyz(pos, Cne);
        matmul3v("N", Cne, gn, ge);
    }

    public static void pregrav(double[] pos, double[] g) {
        double r = norm(pos, 3);
        if (r < IgnavConstants.RE_WGS84 / 2.0) {
            g[0] = g[1] = 0.0;
            g[2] = 9.81;
            return;
        }
        double z = -IgnavConstants.MU / (r * r * r);
        double gamma = 1.5 * IgnavConstants.J2 * SQR(IgnavConstants.RE_WGS84) / SQR(r);
        g[0] = z * (pos[0] + gamma * (1.0 - 5.0 * SQR(pos[2] / r)) * pos[0]);
        g[1] = z * (pos[1] + gamma * (1.0 - 5.0 * SQR(pos[2] / r)) * pos[1]);
        g[2] = z * (pos[2] + gamma * (3.0 - 5.0 * SQR(pos[2] / r)) * pos[2]);
        g[0] += SQR(IgnavConstants.OMGE) * pos[0];
        g[1] += SQR(IgnavConstants.OMGE) * pos[1];
    }

    public static void ecef2pos(double[] r, double[] pos) {
        double e2 = IgnavConstants.FE_WGS84 * (2.0 - IgnavConstants.FE_WGS84);
        double rxy2 = r[0] * r[0] + r[1] * r[1];
        double v = IgnavConstants.RE_WGS84;
        double z = r[2];
        double sinp = 0.0;
        for (int i = 0; i < 10; i++) {
            sinp = z / Math.sqrt(rxy2 + z * z);
            double newV = IgnavConstants.RE_WGS84 / Math.sqrt(1.0 - e2 * sinp * sinp);
            z = r[2] + e2 * newV * sinp;
            if (Math.abs(newV - v) < 1e-12) {
                v = newV;
                break;
            }
            v = newV;
        }
        pos[0] = Math.atan2(z, Math.sqrt(rxy2));
        pos[1] = Math.atan2(r[1], r[0]);
        pos[2] = Math.sqrt(rxy2 + z * z) - v;
    }

    public static void pos2ecef(double[] pos, double[] r) {
        double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);
        double e2 = IgnavConstants.FE_WGS84 * (2.0 - IgnavConstants.FE_WGS84);
        double v = IgnavConstants.RE_WGS84 / Math.sqrt(1.0 - e2 * sinp * sinp);
        r[0] = (v + pos[2]) * cosp * cosl;
        r[1] = (v + pos[2]) * cosp * sinl;
        r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
    }

    public static void xyz2enu(double[] pos, double[] E) {
        double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);
        E[0] = -sinl;        E[1] = cosl;         E[2] = 0.0;
        E[3] = -sinp * cosl;  E[4] = -sinp * sinl;  E[5] = cosp;
        E[6] = cosp * cosl;   E[7] = cosp * sinl;   E[8] = sinp;
    }

    public static void enu2ecef(double[] pos, double[] enu, double[] ecef) {
        double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);
        ecef[0] = -sinl * enu[0] - sinp * cosl * enu[1] + cosp * cosl * enu[2];
        ecef[1] = cosl * enu[0] - sinp * sinl * enu[1] + cosp * sinl * enu[2];
        ecef[2] = cosp * enu[1] + sinp * enu[2];
    }

    public static void ecef2enu(double[] pos, double[] ecef, double[] enu) {
        double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);
        enu[0] = -sinl * ecef[0] + cosl * ecef[1];
        enu[1] = -sinp * cosl * ecef[0] - sinp * sinl * ecef[1] + cosp * ecef[2];
        enu[2] = cosp * cosl * ecef[0] + cosp * sinl * ecef[1] + sinp * ecef[2];
    }

    public static void covenu(double[] pos, double[] P, double[] Q) {
        double[] E = new double[9];
        xyz2enu(pos, E);
        matmul33("NNT", E, P, E, 3, 3, 3, 3, Q);
    }

    public static void covecef(double[] pos, double[] Q, double[] P) {
        double[] E = new double[9];
        xyz2enu(pos, E);
        double[] ET = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                ET[i * 3 + j] = E[j * 3 + i];
            }
        }
        double[] tmp = new double[9];
        matmul("NN", 3, 3, 3, 1.0, ET, Q, 0.0, tmp);
        matmul("NN", 3, 3, 3, 1.0, tmp, E, 0.0, P);
    }

    public static void quatmul(double[] p, double[] q, double[] r) {
        r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
        r[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
        r[2] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
        r[3] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
    }

    public static void quatnormalize(double[] q) {
        double n = norm(q, 4);
        if (n > 0.0) {
            for (int i = 0; i < 4; i++) q[i] /= n;
        }
    }

    public static void normquat(double[] q) {
        double e = 0.5 * (norm(q, 4) - 1.0);
        q[0] = (1.0 - e) * q[0];
        q[1] = (1.0 - e) * q[1];
        q[2] = (1.0 - e) * q[2];
    }

    public static void quatconj(double[] q, double[] qc) {
        qc[0] = q[0];
        qc[1] = -q[1];
        qc[2] = -q[2];
        qc[3] = -q[3];
    }

    public static void quatToRhRotMatrix(double[] q, double[] C) {
        double sqw = q[0] * q[0], sqx = q[1] * q[1];
        double sqy = q[2] * q[2], sqz = q[3] * q[3];
        double invs = 1.0 / (sqx + sqy + sqz + sqw);
        double tmp1, tmp2;

        C[0] = (sqx - sqy - sqz + sqw) * invs;
        C[4] = (-sqx + sqy - sqz + sqw) * invs;
        C[8] = (-sqx - sqy + sqz + sqw) * invs;

        tmp1 = q[1] * q[2]; tmp2 = q[3] * q[0];
        C[3] = 2.0 * (tmp1 + tmp2) * invs;
        C[1] = 2.0 * (tmp1 - tmp2) * invs;

        tmp1 = q[1] * q[3]; tmp2 = q[2] * q[0];
        C[6] = 2.0 * (tmp1 + tmp2) * invs;
        C[2] = 2.0 * (tmp1 - tmp2) * invs;

        tmp1 = q[2] * q[3]; tmp2 = q[1] * q[0];
        C[7] = 2.0 * (tmp1 + tmp2) * invs;
        C[5] = 2.0 * (tmp1 - tmp2) * invs;
    }

    public static void quatrot(double[] q, double[] v, int inv, double[] vo) {
        double[] C = new double[9];
        quatToRhRotMatrix(q, C);
        if (inv != 0) {
            matmul3v("T", C, v, vo);
        } else {
            matmul3v("N", C, v, vo);
        }
    }

    public static double vel2head(double[] vel) {
        return Math.atan2(vel[1], Math.abs(vel[0]) < 1E-4 ? 1E-4 : vel[0]);
    }

    public static void rp2head(double roll, double pitch, double[] gyro, double[] head) {
        double sinh = -gyro[1] * Math.cos(roll) + gyro[2] * Math.sin(roll);
        double cosh = gyro[0] * Math.cos(pitch) + gyro[1] * Math.sin(roll) * Math.sin(pitch)
                + gyro[2] * Math.cos(roll) * Math.sin(pitch);
        head[0] = Math.atan2(sinh, cosh);
    }

    public static double stds(double[] x, int n) {
        if (n <= 1) return 0.0;
        double mean = 0.0;
        for (int i = 0; i < n; i++) mean += x[i];
        mean /= n;
        double s = 0.0;
        for (int i = 0; i < n; i++) s += (x[i] - mean) * (x[i] - mean);
        return Math.sqrt(s / (n - 1));
    }

    public static int chksdri(double[] vel, int n) {
        double[] head = new double[n];
        for (int i = 0; i < n; i++) {
            head[i] = vel2head(new double[]{vel[3 * i], vel[3 * i + 1], vel[3 * i + 2]});
        }
        double hstd = stds(head, n);
        return hstd < 3.0 * IgnavConstants.D2R ? 1 : 0;
    }

    public static void getaccl(double[] fb, double[] Cbe, double[] re, double[] ve, double[] ae) {
        double[] ge = new double[3];
        double[] cori = new double[3];
        double[] fe = new double[3];
        matmul3v("N", Cbe, fb, fe);
        matmul3v("N", OMGE_MAT, ve, cori);
        gravity(re, ge);
        for (int i = 0; i < 3; i++) {
            ae[i] = fe[i] + ge[i] - 2.0 * cori[i];
        }
    }

    public static void cnscl(double[] gyro, double[] accl, double[] phim, double[] dvbm,
                             double[] wm0, double[] vm0, double[] dphim, double[] rotm, double[] scullm) {
        double[] tmp = new double[3];
        double[] tmp1 = new double[3];
        cross3(wm0, gyro, tmp);
        for (int i = 0; i < 3; i++) dphim[i] = tmp[i] / 12.0;
        for (int i = 0; i < 3; i++) phim[i] = gyro[i] + dphim[i];
        cross3(gyro, accl, tmp);
        cross3(vm0, gyro, tmp1);
        for (int i = 0; i < 3; i++) scullm[i] = (tmp[i] + tmp1[i]) / 12.0;
        cross3(gyro, accl, tmp);
        for (int i = 0; i < 3; i++) rotm[i] = 0.5 * tmp[i];
        for (int i = 0; i < 3; i++) dvbm[i] = accl[i] + rotm[i] + scullm[i];
    }

    public static double normang(double ang) {
        while (ang < 0.0) ang += 360.0;
        return ang;
    }

    public static void corratt(double[] dx, double[] C) {
        double[] T = new double[9];
        double[] I = eye(3);
        skewsym3(dx, T);
        for (int i = 0; i < 9; i++)
            I[i] -= T[i];
        matcpy(T, C, 3, 3);
        matmul("NN", 3, 3, 3, 1.0, I, T, 0.0, C);
    }

    public static void rot2dcm(double[] w, double[] C) {
        double a = norm(w, 3);
        double[] Ca = new double[9];
        skewsym3(w, Ca);
        if (a < 1E-12) {
            for (int i = 0; i < 9; i++) C[i] = (i % 4 == 0) ? 1.0 : 0.0;
            return;
        }
        double s = Math.sin(a) / a;
        double c = (1.0 - Math.cos(a)) / (a * a);
        double[] Ca2 = new double[9];
        matmul("NN", 3, 3, 3, 1.0, Ca, Ca, 0.0, Ca2);
        for (int i = 0; i < 9; i++) {
            C[i] = (i % 4 == 0 ? 1.0 : 0.0) + s * Ca[i] + c * Ca2[i];
        }
    }

    public static void so3Log(double[] C, double[] omg, double[] theta) {
        double d = (C[0] + C[4] + C[8] - 1.0) / 2.0;
        if (d > 1.0) d = 1.0;
        if (d < -1.0) d = -1.0;
        double t = Math.acos(d);
        if (Math.abs(t) < 1E-12) {
            omg[0] = omg[1] = omg[2] = 0.0;
            if (theta != null) theta[0] = 0.0;
            return;
        }
        double s = Math.sin(t);
        double f = t / (2.0 * s);
        omg[0] = f * (C[7] - C[5]);
        omg[1] = f * (C[2] - C[6]);
        omg[2] = f * (C[3] - C[1]);
        if (theta != null) theta[0] = t;
    }

    public static void Ry(double angle, double[] R) {
        double c = Math.cos(angle), s = Math.sin(angle);
        R[0] = c;   R[1] = 0.0; R[2] = s;
        R[3] = 0.0; R[4] = 1.0; R[5] = 0.0;
        R[6] = -s;  R[7] = 0.0; R[8] = c;
    }

    public static void Rz(double angle, double[] R) {
        double c = Math.cos(angle), s = Math.sin(angle);
        R[0] = c;   R[1] = -s;  R[2] = 0.0;
        R[3] = s;   R[4] = c;   R[5] = 0.0;
        R[6] = 0.0; R[7] = 0.0; R[8] = 1.0;
    }

    public static void getvn(org.gnss.ignav.ins.data.InsState ins, double[] vn) {
        double[] pos = new double[3];
        double[] Cne = new double[9];
        ecef2pos(ins.re, pos);
        ned2xyz(pos, Cne);
        matmul3v("TN", Cne, ins.ve, vn);
    }
}