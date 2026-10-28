package org.gnss.ignav.ins.common;

import org.gnss.ignav.ins.data.BField;
import org.gnss.ignav.ins.data.BFieldModel;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

/**
 * 用 NOAA/NCEI 官方发布的 WMM2025 参考值做交叉验证。
 *
 * 数据来源：https://www.ncei.noaa.gov/products/world-magnetic-model
 * 参考值文件：data/geomag/WMM2025_TestValues.txt（官方随 WMM2025COF.zip 发布）
 * 系数文件：data/geomag/WMM2025.COF（官方旧版三字段头部格式）
 *
 * 官方参考值的角度保留 2 位小数、强度保留 3 位小数，
 * 因此容差取 1 个最小刻度（0.01 deg / 0.05 nT），用于吸收参考值自身的舍入误差。
 */
class WmmReferenceTest {

    private static final String COF_REL = "geomag/WMM2025.COF";
    private static final String REF_REL = "geomag/WMM2025_TestValues.txt";

    private static final double TOL_DEG = 0.01;
    private static final double TOL_NT = 0.05;

    private static String resolve(String rel) {
        Path[] candidates = {
                Paths.get("..", "data", rel),
                Paths.get("data", rel)
        };
        for (Path p : candidates) {
            if (Files.exists(p)) return p.toString();
        }
        fail("测试数据缺失: data/" + rel);
        return null;
    }

    @Test
    void testReadLegacyWmmCoefficientFile() {
        BFieldModel model = new BFieldModel();
        int ret = GeoMag.readModel(model, resolve(COF_REL));

        assertEquals(1, ret);
        assertEquals(1, model.nmodel);
        assertEquals("WMM-2025", model.name[0]);
        assertEquals(2025.0, model.epoch[0], 1E-9);
        assertEquals(2025.0, model.minyr, 1E-9);
        assertEquals(2030.0, model.maxyr, 1E-9);
        assertEquals(12, model.max1[0]);
        assertEquals(-1.0, model.altmin[0], 1E-9);
        assertEquals(600.0, model.altmax[0], 1E-9);
    }

    @Test
    void testAgainstNoaaReferenceValues() throws IOException {
        BFieldModel model = new BFieldModel();
        assertEquals(1, GeoMag.readModel(model, resolve(COF_REL)));

        List<String> lines = Files.readAllLines(Paths.get(resolve(REF_REL)));
        int rows = 0;
        double maxD = 0.0, maxI = 0.0, maxH = 0.0, maxF = 0.0;

        for (String line : lines) {
            if (line.startsWith("#") || line.trim().isEmpty()) continue;

            String[] p = line.trim().split("\\s+");
            double date = Double.parseDouble(p[0]);
            double altKm = Double.parseDouble(p[1]);
            double latDeg = Double.parseDouble(p[2]);
            double lonDeg = Double.parseDouble(p[3]);
            double dRef = Double.parseDouble(p[4]);
            double iRef = Double.parseDouble(p[5]);
            double hRef = Double.parseDouble(p[6]);
            double fRef = Double.parseDouble(p[10]);

            BField bf = new BField();
            int ret = GeoMag.getFieldComponents(bf, model, altKm, GeoMag.UNITS_KILOMETERS,
                    GeoMag.COORDSYS_GEODETIC, latDeg, lonDeg, date);
            assertEquals(1, ret, "getFieldComponents 失败: " + line);

            maxD = Math.max(maxD, Math.abs(bf.d - dRef));
            maxI = Math.max(maxI, Math.abs(bf.i - iRef));
            maxH = Math.max(maxH, Math.abs(bf.h - hRef));
            maxF = Math.max(maxF, Math.abs(bf.f - fRef));
            rows++;
        }

        assertEquals(100, rows, "官方参考值行数应为 100");
        assertTrue(maxD <= TOL_DEG, "最大磁偏角偏差 " + maxD + " deg 超限");
        assertTrue(maxI <= TOL_DEG, "最大磁倾角偏差 " + maxI + " deg 超限");
        assertTrue(maxH <= TOL_NT, "最大水平强度偏差 " + maxH + " nT 超限");
        assertTrue(maxF <= TOL_NT, "最大总场强度偏差 " + maxF + " nT 超限");
    }

    @Test
    void testAltitudeOutOfRangeRejected() {
        BFieldModel model = new BFieldModel();
        assertEquals(1, GeoMag.readModel(model, resolve(COF_REL)));

        BField bf = new BField();
        assertEquals(0, GeoMag.getFieldComponents(bf, model, 601.0, GeoMag.UNITS_KILOMETERS,
                GeoMag.COORDSYS_GEODETIC, 30.0, 114.0, 2025.0));
    }

    @Test
    void testDateOutOfRangeRejected() {
        BFieldModel model = new BFieldModel();
        assertEquals(1, GeoMag.readModel(model, resolve(COF_REL)));

        BField bf = new BField();
        assertEquals(0, GeoMag.getFieldComponents(bf, model, 0.0, GeoMag.UNITS_KILOMETERS,
                GeoMag.COORDSYS_GEODETIC, 30.0, 114.0, 2031.5));
    }
}