package org.gnss.ignav.ins.common;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class InsMathTest {

    private static final double EPS = 1E-9;

    @Test
    void testNorm() {
        double[] v = {3.0, 4.0};
        assertEquals(5.0, InsMath.norm(v, 2), EPS);

        double[] v3 = {1.0, 2.0, 2.0};
        assertEquals(3.0, InsMath.norm(v3, 3), EPS);

        double[] v0 = {0.0, 0.0, 0.0};
        assertEquals(0.0, InsMath.norm(v0, 3), EPS);
    }

    @Test
    void testMatmul() {
        double[] A = {1, 2, 3, 4, 5, 6};
        double[] B = {1, 1, 1, 1, 1, 1};
        double[] C = new double[4];

        InsMath.matmul("NN", 2, 2, 3, 1.0, A, B, 0.0, C);

        assertEquals(6.0, C[0], EPS);
        assertEquals(6.0, C[1], EPS);
        assertEquals(15.0, C[2], EPS);
        assertEquals(15.0, C[3], EPS);
    }

    @Test
    void testMatmulIdentity() {
        double[] I = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        double[] A = {1, 2, 3, 4, 5, 6, 7, 8, 9};
        double[] C = new double[9];

        InsMath.matmul("NN", 3, 3, 3, 1.0, I, A, 0.0, C);

        for (int i = 0; i < 9; i++) {
            assertEquals(A[i], C[i], EPS);
        }
    }

    @Test
    void testEcef2posPos2ecef() {
        double[] rr = {-2267749.234, 5009389.567, 3221290.123};
        double[] pos = new double[3];
        double[] rr2 = new double[3];

        InsMath.ecef2pos(rr, pos);
        InsMath.pos2ecef(pos, rr2);

        for (int i = 0; i < 3; i++) {
            assertEquals(rr[i], rr2[i], 0.1);
        }
    }

    @Test
    void testEcef2posKnownLocation() {
        double[] rr = {-2267749.234, 5009389.567, 3221290.123};
        double[] pos = new double[3];

        InsMath.ecef2pos(rr, pos);

        assertTrue(pos[0] > 0 && pos[0] < IgnavConstants.D2R * 90);
        assertTrue(Math.abs(pos[1]) < IgnavConstants.D2R * 180);
    }

    @Test
    void testSkewsym3() {
        double[] w = {1.0, 2.0, 3.0};
        double[] S = new double[9];
        InsMath.skewsym3(w, S);

        assertEquals(0.0, S[0], EPS);
        assertEquals(-3.0, S[1], EPS);
        assertEquals(2.0, S[2], EPS);
        assertEquals(3.0, S[3], EPS);
        assertEquals(0.0, S[4], EPS);
        assertEquals(-1.0, S[5], EPS);
        assertEquals(-2.0, S[6], EPS);
        assertEquals(1.0, S[7], EPS);
        assertEquals(0.0, S[8], EPS);
    }

    @Test
    void testEnu2ecefEcef2enu() {
        double[] pos = {39.9 * IgnavConstants.D2R, 116.4 * IgnavConstants.D2R, 50.0};
        double[] enu = {100.0, 200.0, 300.0};
        double[] ecef = new double[3];
        double[] enu2 = new double[3];

        InsMath.enu2ecef(pos, enu, ecef);
        InsMath.ecef2enu(pos, ecef, enu2);

        for (int i = 0; i < 3; i++) {
            assertEquals(enu[i], enu2[i], 1E-6);
        }
    }

    @Test
    void testCovenuCovecef() {
        double[] pos = {39.9 * IgnavConstants.D2R, 116.4 * IgnavConstants.D2R, 50.0};
        double[] P = new double[9];
        P[0] = 1.0; P[4] = 2.0; P[8] = 3.0;
        P[1] = P[3] = 0.1;
        P[2] = P[6] = 0.2;
        P[5] = P[7] = 0.3;

        double[] Q = new double[9];
        InsMath.covenu(pos, P, Q);

        double[] P2 = new double[9];
        InsMath.covecef(pos, Q, P2);

        for (int i = 0; i < 9; i++) {
            assertEquals(P[i], P2[i], 1E-9);
        }
    }

    @Test
    void testRot2dcm() {
        double[] w = {0.0, 0.0, 0.0};
        double[] C = new double[9];
        InsMath.rot2dcm(w, C);

        assertEquals(1.0, C[0], EPS);
        assertEquals(0.0, C[1], EPS);
        assertEquals(1.0, C[4], EPS);
        assertEquals(1.0, C[8], EPS);
    }

    @Test
    void testRy() {
        double[] R = new double[9];
        InsMath.Ry(0.0, R);
        assertEquals(1.0, R[0], EPS);
        assertEquals(1.0, R[4], EPS);
        assertEquals(1.0, R[8], EPS);
    }

    @Test
    void testRz() {
        double[] R = new double[9];
        InsMath.Rz(0.0, R);
        assertEquals(1.0, R[0], EPS);
        assertEquals(1.0, R[4], EPS);
        assertEquals(1.0, R[8], EPS);
    }

    @Test
    void testMatinv() {
        double[] A = {4, 2, 2, 3};
        double[] Ainv = A.clone();
        int ret = InsMath.matinv(Ainv, 2);

        assertEquals(0, ret);
        double det = A[0] * A[3] - A[1] * A[2];
        assertEquals(A[3] / det, Ainv[0], EPS);
        assertEquals(-A[1] / det, Ainv[1], EPS);
        assertEquals(-A[2] / det, Ainv[2], EPS);
        assertEquals(A[0] / det, Ainv[3], EPS);
    }

    @Test
    void testMatinvIdentity() {
        double[] I = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        double[] Iinv = I.clone();
        int ret = InsMath.matinv(Iinv, 3);

        assertEquals(0, ret);
        for (int i = 0; i < 9; i++) {
            assertEquals(I[i], Iinv[i], EPS);
        }
    }

    @Test
    void testGTimeDiff() {
        org.gnss.ignav.ins.data.GTime t1 = new org.gnss.ignav.ins.data.GTime(1000, 0.5);
        org.gnss.ignav.ins.data.GTime t2 = new org.gnss.ignav.ins.data.GTime(1000, 0.0);

        assertEquals(0.5, org.gnss.ignav.ins.data.GTime.timeDiff(t1, t2), EPS);
    }
}