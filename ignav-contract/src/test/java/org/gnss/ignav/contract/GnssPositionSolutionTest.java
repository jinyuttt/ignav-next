package org.gnss.ignav.contract;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class GnssPositionSolutionTest {

    @Test
    void testDefaultConstructor() {
        GnssPositionSolution result = new GnssPositionSolution();
        assertNotNull(result.getTime());
        assertNotNull(result.getPosEcef());
        assertNotNull(result.getVelEcef());
        assertNotNull(result.getPosCov());
        assertNotNull(result.getVelCov());
        assertEquals(3, result.getPosEcef().length);
        assertEquals(3, result.getVelEcef().length);
        assertEquals(9, result.getPosCov().length);
        assertEquals(GnssPositionSolution.SolutionStatus.NONE, result.getSolStatus());
    }

    @Test
    void testIsValid() {
        GnssPositionSolution result = new GnssPositionSolution();
        assertFalse(result.isValid());

        result.setSolStatus(GnssPositionSolution.SolutionStatus.FIX);
        assertTrue(result.isValid());

        result.setSolStatus(GnssPositionSolution.SolutionStatus.SPP);
        assertTrue(result.isValid());
    }

    @Test
    void testIsFixed() {
        GnssPositionSolution result = new GnssPositionSolution();
        assertFalse(result.isFixed());

        result.setSolStatus(GnssPositionSolution.SolutionStatus.FIX);
        assertTrue(result.isFixed());

        result.setSolStatus(GnssPositionSolution.SolutionStatus.FLOAT);
        assertFalse(result.isFixed());
    }

    @Test
    void testSetGetVelocity() {
        GnssPositionSolution result = new GnssPositionSolution();
        double[] vel = {10.0, 20.0, 30.0};
        result.setVelEcef(vel);

        assertEquals(vel[0], result.getVelEcef()[0], 1E-12);
        assertEquals(vel[1], result.getVelEcef()[1], 1E-12);
        assertEquals(vel[2], result.getVelEcef()[2], 1E-12);
    }

    @Test
    void testSolutionStatusFromCode() {
        assertEquals(GnssPositionSolution.SolutionStatus.NONE, GnssPositionSolution.SolutionStatus.fromCode(0));
        assertEquals(GnssPositionSolution.SolutionStatus.FIX, GnssPositionSolution.SolutionStatus.fromCode(1));
        assertEquals(GnssPositionSolution.SolutionStatus.FLOAT, GnssPositionSolution.SolutionStatus.fromCode(2));
        assertEquals(GnssPositionSolution.SolutionStatus.SPP, GnssPositionSolution.SolutionStatus.fromCode(3));
        assertEquals(GnssPositionSolution.SolutionStatus.OTHER, GnssPositionSolution.SolutionStatus.fromCode(99));
    }

    @Test
    void testRoundTrip() {
        GnssPositionSolution result = new GnssPositionSolution();
        result.setTime(new GTime(1000, 0.5));
        result.setPosEcef(new double[]{-2267749.234, 5009389.567, 3221290.123});
        result.setVelEcef(new double[]{10.0, 20.0, 30.0});
        result.setNumSat(12);
        result.setSolStatus(GnssPositionSolution.SolutionStatus.FIX);
        result.setPosCov(new double[]{0.01, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.03});

        GnssPositionSolution result2 = new GnssPositionSolution();
        result2.setTime(result.getTime());
        result2.setPosEcef(result.getPosEcef().clone());
        result2.setVelEcef(result.getVelEcef().clone());
        result2.setNumSat(result.getNumSat());
        result2.setSolStatus(result.getSolStatus());
        result2.setPosCov(result.getPosCov().clone());

        assertEquals(result.getTime().time, result2.getTime().time);
        assertEquals(result.getTime().sec, result2.getTime().sec, 1E-12);
        assertEquals(result.getPosEcef()[0], result2.getPosEcef()[0], 1E-6);
        assertEquals(result.getVelEcef()[0], result2.getVelEcef()[0], 1E-6);
        assertEquals(result.getNumSat(), result2.getNumSat());
        assertEquals(result.getSolStatus(), result2.getSolStatus());
        assertEquals(result.getPosCov()[0], result2.getPosCov()[0], 1E-6);
    }
}