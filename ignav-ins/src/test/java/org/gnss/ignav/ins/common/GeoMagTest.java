package org.gnss.ignav.ins.common;

import org.gnss.ignav.ins.data.BField;
import org.gnss.ignav.ins.data.BFieldModel;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.gnss.ignav.ins.data.Magd;
import org.gnss.ignav.ins.data.GTime;
import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.aiding.InsMagnetometer;
import org.gnss.ignav.ins.aiding.InsStateIdx;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class GeoMagTest {

    private static final double EPS = 1E-6;
    private static final double EPS_DEG = 0.1;

    @Test
    void testJulday() {
        double jd = GeoMag.julday(1, 1, 2020);
        assertTrue(jd >= 2020.0 && jd < 2020.01, "Jan 1 should be near 2020.0, got " + jd);

        double jd2 = GeoMag.julday(7, 1, 2020);
        assertTrue(jd2 > 2020.0 && jd2 < 2021.0);

        double jd3 = GeoMag.julday(12, 31, 2020);
        assertTrue(jd3 >= 2020.99 && jd3 <= 2021.0, "Dec 31 should be near 2021.0, got " + jd3);
    }

    @Test
    void testTime2EpochLeapYear() {
        long[] unix = {946684800L, 1577836800L, 1583020800L, 1609459200L, 1672531200L};
        String[] want = {"2000-01-01", "2020-01-01", "2020-03-01", "2021-01-01", "2023-01-01"};
        for (int i = 0; i < unix.length; i++) {
            GTime t = new GTime(unix[i] - IgnavConstants.GPST0_TIME, 0.0);
            double[] ep = new double[6];
            GTime.time2epoch(t, ep);
            String got = String.format("%04d-%02d-%02d", (int) ep[0], (int) ep[1], (int) ep[2]);
            assertEquals(want[i], got, "time2epoch mismatch for unix=" + unix[i]);
        }

        GTime t = new GTime(1577836800L - IgnavConstants.GPST0_TIME, 3600.0 * 12 + 34 * 60 + 56.0);
        double[] ep = new double[6];
        GTime.time2epoch(t, ep);
        assertEquals(12, (int) ep[3]);
        assertEquals(34, (int) ep[4]);
        assertEquals(56.0, ep[5], EPS);
    }

    @Test
    void testReadModelWMM() {
        BFieldModel model = new BFieldModel();
        String wmmFile = "src/main/resources/geomag/WMM2015.COF";
        int ret = GeoMag.readModel(model, wmmFile);
        assertEquals(1, ret);
        assertTrue(model.nmodel > 0);
        assertTrue(model.minyr > 2010);
        assertTrue(model.maxyr >= 2020);
    }

    @Test
    void testReadModelIGRF() {
        BFieldModel model = new BFieldModel();
        String igrfFile = "src/main/resources/geomag/IGRF12.COF";
        int ret = GeoMag.readModel(model, igrfFile);
        assertEquals(1, ret);
        assertTrue(model.nmodel > 1);
        assertTrue(model.minyr < 1950);
        assertTrue(model.maxyr >= 2020);
    }

    @Test
    void testGetFieldComponentsWMM() {
        BFieldModel model = new BFieldModel();
        String wmmFile = "src/main/resources/geomag/WMM2015.COF";
        GeoMag.readModel(model, wmmFile);

        BField bfield = new BField();
        double sdate = GeoMag.julday(1, 1, 2020);
        int ret = GeoMag.getFieldComponents(bfield, model, 0.0, GeoMag.UNITS_METERS,
                GeoMag.COORDSYS_GEODETIC, 30.0, 114.0, sdate);

        assertEquals(1, ret);
        assertTrue(Math.abs(bfield.d) < 30.0, "declination should be within +/-30 deg, got " + bfield.d);
        assertTrue(bfield.f > 20000.0 && bfield.f < 70000.0, "total field should be 20k-70k nT, got " + bfield.f);
        assertTrue(bfield.h > 10000.0, "horizontal intensity should be >10k nT, got " + bfield.h);
        assertFalse(Double.isNaN(bfield.d), "declination should not be NaN");
    }

    @Test
    void testGetFieldComponentsIGRF() {
        BFieldModel model = new BFieldModel();
        String igrfFile = "src/main/resources/geomag/IGRF12.COF";
        GeoMag.readModel(model, igrfFile);

        BField bfield = new BField();
        double sdate = GeoMag.julday(1, 1, 2020);
        int ret = GeoMag.getFieldComponents(bfield, model, 0.0, GeoMag.UNITS_METERS,
                GeoMag.COORDSYS_GEODETIC, 30.0, 114.0, sdate);

        assertEquals(1, ret);
        assertTrue(Math.abs(bfield.d) < 30.0, "declination should be within +/-30 deg, got " + bfield.d);
        assertTrue(bfield.f > 20000.0 && bfield.f < 70000.0, "total field should be 20k-70k nT, got " + bfield.f);
    }

    @Test
    void testDeclinationWuhan() {
        BFieldModel model = new BFieldModel();
        String wmmFile = "src/main/resources/geomag/WMM2015.COF";
        GeoMag.readModel(model, wmmFile);

        BField bfield = new BField();
        double sdate = GeoMag.julday(1, 1, 2020);
        GeoMag.getFieldComponents(bfield, model, 0.0, GeoMag.UNITS_METERS,
                GeoMag.COORDSYS_GEODETIC, 30.5, 114.3, sdate);

        assertTrue(Math.abs(bfield.d) < 15.0, "Wuhan declination should be small, got " + bfield.d);
        assertTrue(bfield.i > 30.0 && bfield.i < 60.0, "Wuhan inclination should be 30-60 deg, got " + bfield.i);
    }

    @Test
    void testInvalidDate() {
        BFieldModel model = new BFieldModel();
        String wmmFile = "src/main/resources/geomag/WMM2015.COF";
        GeoMag.readModel(model, wmmFile);

        BField bfield = new BField();
        int ret = GeoMag.getFieldComponents(bfield, model, 0.0, GeoMag.UNITS_METERS,
                GeoMag.COORDSYS_GEODETIC, 30.0, 114.0, 1900.0);

        assertEquals(0, ret);
    }

    @Test
    void testInvalidCoordinates() {
        BFieldModel model = new BFieldModel();
        String wmmFile = "src/main/resources/geomag/WMM2015.COF";
        GeoMag.readModel(model, wmmFile);

        BField bfield = new BField();
        double sdate = GeoMag.julday(1, 1, 2020);
        int ret = GeoMag.getFieldComponents(bfield, model, 0.0, GeoMag.UNITS_METERS,
                GeoMag.COORDSYS_GEODETIC, 91.0, 0.0, sdate);

        assertEquals(0, ret);
    }

    @Test
    void testMagmodelStatic() {
        String wmmFile = "src/main/resources/geomag/WMM2015.COF";
        int ret = GeoMag.magmodel(wmmFile);
        assertEquals(1, ret);

        BFieldModel model = GeoMag.getMagModel();
        assertTrue(model.nmodel > 0);
    }

    @Test
    void testMagnetometerInvalidData() {
        InsOpt opt = new InsOpt();
        opt.baopt = IgnavConstants.INS_BAEST;
        opt.bgopt = IgnavConstants.INS_BGEST;

        InsState ins = new InsState();
        ins.nx = InsStateIdx.nx(opt);
        ins.x = new double[ins.nx];
        ins.P = new double[ins.nx * ins.nx];

        Magd magData = new Magd();
        magData.val[0] = 0.0;
        magData.val[1] = 0.0;
        magData.val[2] = 0.0;

        int ret = InsMagnetometer.magnetometer(ins, opt, magData);
        assertEquals(0, ret);
    }

    @Test
    void testInsOptMagOpt() {
        InsOpt opt = new InsOpt();
        assertNotNull(opt.magopt);
        assertEquals(1.0, opt.magopt.sx);
        assertEquals(1.0, opt.magopt.sy);
        assertEquals(1.0, opt.magopt.sz);
        assertEquals(0.0, opt.magopt.ox);
        assertEquals(0.0, opt.magopt.oy);
        assertEquals(0.0, opt.magopt.oz);
    }

    @Test
    void testMagdCopy() {
        Magd mag1 = new Magd();
        mag1.time = new GTime(1000, 0.5);
        mag1.val[0] = 1.0;
        mag1.val[1] = 2.0;
        mag1.val[2] = 3.0;

        Magd mag2 = new Magd(mag1);
        assertEquals(mag1.time.time, mag2.time.time);
        assertEquals(mag1.val[0], mag2.val[0], EPS);
        assertEquals(mag1.val[1], mag2.val[1], EPS);
        assertEquals(mag1.val[2], mag2.val[2], EPS);
    }

    @Test
    void testBFieldDefaults() {
        BField bf = new BField();
        assertEquals(0.0, bf.d);
        assertEquals(0.0, bf.f);
        assertEquals(0.0, bf.x);
        assertEquals(0.0, bf.y);
        assertEquals(0.0, bf.z);
    }

    @Test
    void testBFieldModelDefaults() {
        BFieldModel model = new BFieldModel();
        assertEquals(0, model.nmodel);
        assertEquals(0.0, model.minyr);
        assertEquals(0.0, model.maxyr);
        assertEquals(BFieldModel.MAXMOD, model.name.length);
        assertEquals(BFieldModel.MAXCOEFF, model.gh1[0].length);
    }
}