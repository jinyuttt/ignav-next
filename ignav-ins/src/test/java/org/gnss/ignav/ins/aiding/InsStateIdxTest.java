package org.gnss.ignav.ins.aiding;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.data.InsOpt;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class InsStateIdxTest {

    private InsOpt createDefaultOpt() {
        InsOpt opt = new InsOpt();
        opt.baopt = IgnavConstants.INS_BAEST;
        opt.bgopt = IgnavConstants.INS_BGEST;
        return opt;
    }

    @Test
    void testXnX() {
        InsOpt opt = createDefaultOpt();
        int nx = InsStateIdx.nx(opt);
        assertTrue(nx > 0);
        assertEquals(15, nx);
    }

    @Test
    void testXnXMinimal() {
        InsOpt opt = new InsOpt();
        int nx = InsStateIdx.nx(opt);
        assertEquals(9, nx);
    }

    @Test
    void testXiV() {
        InsOpt opt = createDefaultOpt();
        int iv = InsStateIdx.xiV(opt);
        assertEquals(3, iv);
    }

    @Test
    void testXiP() {
        InsOpt opt = createDefaultOpt();
        int ip = InsStateIdx.xiP(opt);
        assertEquals(6, ip);
    }

    @Test
    void testXiBa() {
        InsOpt opt = createDefaultOpt();
        int iba = InsStateIdx.xiBa(opt);
        assertEquals(12, iba);
    }

    @Test
    void testXiBg() {
        InsOpt opt = createDefaultOpt();
        int ibg = InsStateIdx.xiBg(opt);
        assertEquals(9, ibg);
    }

    @Test
    void testXnBa() {
        InsOpt opt = createDefaultOpt();
        assertEquals(3, InsStateIdx.xnBa(opt));
    }

    @Test
    void testXnBg() {
        InsOpt opt = createDefaultOpt();
        assertEquals(3, InsStateIdx.xnBg(opt));
    }

    @Test
    void testXnA() {
        InsOpt opt = createDefaultOpt();
        assertEquals(3, InsStateIdx.xnA(opt));
    }

    @Test
    void testXiA() {
        InsOpt opt = createDefaultOpt();
        assertEquals(0, InsStateIdx.xiA(opt));
    }
}