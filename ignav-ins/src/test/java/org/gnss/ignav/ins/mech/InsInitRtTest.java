package org.gnss.ignav.ins.mech;

import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.data.GTime;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class InsInitRtTest {

    private InsOpt createDefaultOpt() {
        InsOpt opt = new InsOpt();
        opt.baopt = IgnavConstants.INS_BAEST;
        opt.bgopt = IgnavConstants.INS_BGEST;
        return opt;
    }

    @Test
    void testInsStateCreation() {
        InsState ins = new InsState();
        assertNotNull(ins.re);
        assertNotNull(ins.ve);
        assertNotNull(ins.Cbe);
        assertEquals(3, ins.re.length);
        assertEquals(3, ins.ve.length);
        assertEquals(9, ins.Cbe.length);
    }

    @Test
    void testInsStateDefaultStatus() {
        InsState ins = new InsState();
        assertEquals(IgnavConstants.INSS_NONE, ins.stat);
    }

    @Test
    void testInsInitWithPosVel() {
        InsOpt opt = createDefaultOpt();
        InsState ins = new InsState();

        double[] pos = {-2267749.234, 5009389.567, 3221290.123};
        double[] vel = {10.0, 20.0, 5.0};
        GTime time = new GTime(IgnavConstants.GPST0_TIME + 1000, 0.0);

        Imud imu = new Imud();
        imu.time = new GTime(time);
        imu.gyro = new double[]{0.0, 0.0, 0.0};
        imu.accl = new double[]{0.0, 0.0, 9.8};

        int ret = InsInitRt.insinirt(pos, vel, time, imu, opt, ins);
        assertTrue(ret == 0 || ret == 1);
    }

    @Test
    void testInsOptDefaults() {
        InsOpt opt = new InsOpt();
        assertNotNull(opt);
    }

    @Test
    void testImudCreation() {
        Imud imu = new Imud();
        assertNotNull(imu.gyro);
        assertNotNull(imu.accl);
        assertEquals(3, imu.gyro.length);
        assertEquals(3, imu.accl.length);
    }

    @Test
    void testGTimeOperations() {
        GTime t1 = new GTime(1000, 0.5);
        GTime t2 = new GTime(1000, 0.0);

        assertEquals(0.5, GTime.timeDiff(t1, t2), 1E-12);
        assertEquals(-0.5, GTime.timeDiff(t2, t1), 1E-12);
    }

    @Test
    void testConstantsConsistency() {
        assertTrue(IgnavConstants.RE_WGS84 > 0);
        assertTrue(IgnavConstants.FE_WGS84 > 0 && IgnavConstants.FE_WGS84 < 1);
        assertTrue(IgnavConstants.OMGE > 0);
        assertTrue(IgnavConstants.GPST0_TIME > 0);
        assertTrue(IgnavConstants.D2R > 0);
        assertTrue(IgnavConstants.R2D > 0);
        assertEquals(IgnavConstants.D2R * IgnavConstants.R2D, 1.0, 1E-12);
    }
}