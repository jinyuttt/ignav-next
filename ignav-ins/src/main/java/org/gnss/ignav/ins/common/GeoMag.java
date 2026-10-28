package org.gnss.ignav.ins.common;

import org.gnss.ignav.ins.data.BField;
import org.gnss.ignav.ins.data.BFieldModel;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public final class GeoMag {

    private static final Logger logger = LoggerFactory.getLogger(GeoMag.class);

    private static final double EARTH_RADIUS = 6371.2;
    private static final double FT2KM = 1.0 / 0.0003048;
    private static final double PI = 3.141592654;
    private static final double RAD2DEG = 180.0 / PI;
    private static final int IEXT = 0;
    private static final int RECL = 81;
    private static final int MAXINBUFF = RECL + 14;
    private static final int MAXREAD = MAXINBUFF - 2;
    private static final double EXT_COEFF1 = 0.0;
    private static final double EXT_COEFF2 = 0.0;
    private static final double EXT_COEFF3 = 0.0;

    private static final int WMM_DEGREE = 12;
    private static final double WMM_VALID_YEARS = 5.0;
    private static final double WMM_ALT_MIN_KM = -1.0;
    private static final double WMM_ALT_MAX_KM = 600.0;

    public static final int UNITS_KILOMETERS = 1;
    public static final int UNITS_METERS = 2;
    public static final int UNITS_FEET = 3;

    public static final int COORDSYS_GEODETIC = 1;
    public static final int COORDSYS_GEOCENTRIC = 2;

    private static BFieldModel magModel = new BFieldModel();
    private static BField mag = new BField();

    private GeoMag() {}

    public static BFieldModel getMagModel() {
        return magModel;
    }

    public static BField getMag() {
        return mag;
    }

    public static int magmodel(String file) {
        logger.debug("magmodel, file={}", file);
        return readModel(magModel, file);
    }

    /**
     * 计算给定位置与时刻的地磁要素。
     *
     * 单位约定：{@code alt} 按 {@code altUnits} 解释（km / m / ft）；
     * {@code latitude}、{@code longitude} 为 deg；{@code sdate} 为十进制年。
     * 输出 {@code bfield}：x/y/z 与 h/f 单位为 nT，d（磁偏角）与 i（磁倾角）单位为 deg。
     *
     * @return 1 成功，0 表示日期或高度/经纬度超出模型适用范围
     */
    public static int getFieldComponents(BField bfield, BFieldModel model, double alt, int altUnits,
                                         int coordSys, double latitude, double longitude, double sdate) {
        int warn_H = 0, warn_H_strong = 0, warn_P = 0;
        int modelI = 0;
        int nmax;
        double warn_H_val = 99999.0, warn_H_strong_val = 99999.0;
        double minAlt = 0.0, maxAlt = 0.0;
        double dtemp = 0.0, ftemp = 0.0, htemp = 0.0, itemp = 0.0;
        double xtemp = 0.0, ytemp = 0.0, ztemp = 0.0;
        double[] gha = new double[BFieldModel.MAXCOEFF];
        double[] ghb = new double[BFieldModel.MAXCOEFF];

        if ((sdate > model.maxyr) && (sdate < model.maxyr + 1)) {
            logger.warn("The date {} is out of range, but still within one year of model expiration date.", sdate);
        }
        if (sdate < model.minyr || sdate > model.maxyr + 1) {
            return 0;
        }

        for (modelI = 0; modelI < model.nmodel; modelI++) {
            if (sdate < model.yrmax[modelI]) break;
        }
        if (modelI == model.nmodel) modelI = model.nmodel - 1;

        minAlt = model.altmin[modelI];
        maxAlt = model.altmax[modelI];
        if (alt < minAlt || alt > maxAlt) {
            return 0;
        }

        switch (altUnits) {
        case UNITS_METERS:
            alt *= 0.001;
            break;
        case UNITS_FEET:
            alt /= FT2KM;
            break;
        default:
            break;
        }
        if (latitude < -90 || latitude > 90 || longitude < -180 || longitude > 180) {
            return 0;
        }

        if (model.max2[modelI] == 0) {
            nmax = interpsh(sdate, model.yrmin[modelI], model.max1[modelI], model.yrmin[modelI + 1],
                    model.max1[modelI + 1], 3, model.gh1[modelI], model.gh2[modelI], gha, ghb);
            nmax = interpsh(sdate + 1, model.yrmin[modelI], model.max1[modelI], model.yrmin[modelI + 1],
                    model.max1[modelI + 1], 4, model.gh1[modelI], model.gh2[modelI], gha, ghb);
        } else {
            nmax = extrapsh(sdate, model.epoch[modelI], model.max1[modelI], model.max2[modelI], 3,
                    model.gh1[modelI], model.gh2[modelI], gha, ghb);
            nmax = extrapsh(sdate + 1, model.epoch[modelI], model.max1[modelI], model.max2[modelI], 4,
                    model.gh1[modelI], model.gh2[modelI], gha, ghb);
        }

        double[] xz = new double[1], yz = new double[1], zz = new double[1];
        shval3(coordSys, latitude, longitude, alt, nmax, 3, IEXT, EXT_COEFF1, EXT_COEFF2, EXT_COEFF3,
                gha, ghb, xz, yz, zz);
        bfield.x = xz[0]; bfield.y = yz[0]; bfield.z = zz[0];
        dihf(bfield.x, bfield.y, bfield.z, bfield);

        double[] xt = new double[1], yt = new double[1], zt = new double[1];
        shval3(coordSys, latitude, longitude, alt, nmax, 4, IEXT, EXT_COEFF1, EXT_COEFF2, EXT_COEFF3,
                gha, ghb, xt, yt, zt);
        xtemp = xt[0]; ytemp = yt[0]; ztemp = zt[0];

        double[] dArr = new double[1], iArr = new double[1], hArr = new double[1], fArr = new double[1];
        dihfCalc(xtemp, ytemp, ztemp, dArr, iArr, hArr, fArr);
        dtemp = dArr[0]; itemp = iArr[0]; htemp = hArr[0]; ftemp = fArr[0];

        bfield.ddot = ((dtemp - bfield.d) * RAD2DEG);
        if (bfield.ddot > 180.0) bfield.ddot -= 360.0;
        if (bfield.ddot <= -180.0) bfield.ddot += 360.0;
        bfield.ddot *= 60.0;

        bfield.idot = ((itemp - bfield.i) * RAD2DEG) * 60;
        bfield.d *= RAD2DEG;
        bfield.i *= RAD2DEG;
        bfield.hdot = htemp - bfield.h;
        bfield.xdot = xtemp - bfield.x;
        bfield.ydot = ytemp - bfield.y;
        bfield.zdot = ztemp - bfield.z;
        bfield.fdot = ftemp - bfield.f;

        if (bfield.h < 100.0) {
            bfield.d = Double.NaN;
            bfield.ddot = Double.NaN;
        }
        if (bfield.h < 1000.0) {
            warn_H = 0;
            warn_H_strong = 1;
            if (bfield.h < warn_H_strong_val) {
                warn_H_strong_val = bfield.h;
            }
        } else if (bfield.h < 5000.0 && warn_H_strong == 0) {
            warn_H = 1;
            if (bfield.h < warn_H_val) {
                warn_H_val = bfield.h;
            }
        }
        if (90.0 - Math.abs(latitude) <= 0.001) {
            bfield.x = Double.NaN;
            bfield.y = Double.NaN;
            bfield.d = Double.NaN;
            bfield.xdot = Double.NaN;
            bfield.ydot = Double.NaN;
            bfield.ddot = Double.NaN;
            warn_P = 1;
            warn_H = 0;
            warn_H_strong = 0;
        }
        return 1;
    }

    /**
     * 由日历日期计算十进制年（decimal year），供地磁模型时间插值/外推使用。
     *
     * @param month 月份，1–12
     * @param day   日，1–31
     * @param year  四位年份
     * @return 十进制年，例如 2020-01-01 返回 2020.0027
     */
    public static double julday(int month, int day, int year) {
        int[] days = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
        int leapYear = (((year % 4) == 0) && (((year % 100) != 0) || ((year % 400) == 0))) ? 1 : 0;
        double dayInYear = (days[month - 1] + day + (month > 2 ? leapYear : 0));
        return ((double) year + (dayInYear / (365.0 + leapYear)));
    }

    /**
     * 读取地磁模型系数文件。
     *
     * 支持两种头部格式：内部 9 字段格式（IGRF 多纪元表）与 NOAA 官方 WMM.COF 的
     * 3 字段旧格式。数据行统一为 {@code n m g h dg dh}，其中 g/h 单位为 nT，
     * dg/dh 单位为 nT/yr。
     *
     * @return 1 成功，0 失败
     */
    public static int readModel(BFieldModel model, String mdfile) {
        int lineNum = 0;
        int modelI = -1;

        java.util.List<double[][]> allGh1 = new java.util.ArrayList<>();
        java.util.List<double[][]> allGh2 = new java.util.ArrayList<>();
        java.util.List<int[]> modelInfo = new java.util.ArrayList<>();

        double[] curGh1 = null;
        double[] curGh2 = null;
        int curMax1 = 0;
        int curMax2 = 0;

        try (BufferedReader reader = new BufferedReader(new FileReader(mdfile))) {
            String line;
            while ((line = reader.readLine()) != null) {
                lineNum++;

                if (line.length() >= 3 && line.substring(0, 3).equals("   ")) {
                    if (modelI >= 0 && curGh1 != null) {
                        allGh1.add(new double[][]{curGh1});
                        allGh2.add(new double[][]{curGh2});
                        modelInfo.add(new int[]{curMax1, curMax2});
                    }

                    modelI++;
                    if (modelI >= BFieldModel.MAXMOD) {
                        logger.error("Too many models in file {} on line {}.", mdfile, lineNum);
                        return 0;
                    }

                    String[] parts = line.trim().split("\\s+");
                    try {
                        if (parts.length < 9) {
                            if (!parseLegacyWmmHeader(model, modelI, parts)) {
                                logger.error("Unsupported model header on line {} in {}.", lineNum, mdfile);
                                return 0;
                            }
                        } else {
                            model.name[modelI] = parts[0];
                            model.epoch[modelI] = Double.parseDouble(parts[1]);
                            model.max1[modelI] = Integer.parseInt(parts[2]);
                            model.max2[modelI] = Integer.parseInt(parts[3]);
                            model.max3[modelI] = Integer.parseInt(parts[4]);
                            model.yrmin[modelI] = Double.parseDouble(parts[5]);
                            model.yrmax[modelI] = Double.parseDouble(parts[6]);
                            model.altmin[modelI] = Double.parseDouble(parts[7]);
                            model.altmax[modelI] = Double.parseDouble(parts[8]);
                        }
                    } catch (Exception e) {
                        logger.error("Error parsing header line {} in {}: {}", lineNum, mdfile, e.getMessage());
                        return 0;
                    }

                    curMax1 = model.max1[modelI];
                    curMax2 = model.max2[modelI];
                    curGh1 = new double[BFieldModel.MAXCOEFF];
                    curGh2 = new double[BFieldModel.MAXCOEFF];
                } else {
                    if (modelI < 0) continue;

                    String trimmed = line.trim();
                    if (trimmed.isEmpty()) continue;

                    String[] parts = trimmed.split("\\s+");
                    if (parts.length < 4) continue;

                    try {
                        int n = Integer.parseInt(parts[0]);
                        int m = Integer.parseInt(parts[1]);
                        double g = Double.parseDouble(parts[2]);
                        double h = Double.parseDouble(parts[3]);
                        double svG = 0.0, svH = 0.0;
                        if (curMax2 != 0 && parts.length >= 6) {
                            svG = Double.parseDouble(parts[4]);
                            svH = Double.parseDouble(parts[5]);
                        }

                        int ii;
                        if (m == 0) {
                            ii = n * n;
                        } else {
                            ii = n * n + 2 * m - 1;
                        }

                        if (ii > 0 && ii < BFieldModel.MAXCOEFF) {
                            curGh1[ii] = g;
                            if (curMax2 != 0) {
                                curGh2[ii] = svG;
                            }
                        }

                        if (m != 0) {
                            int ii_h = n * n + 2 * m;
                            if (ii_h > 0 && ii_h < BFieldModel.MAXCOEFF) {
                                curGh1[ii_h] = h;
                                if (curMax2 != 0) {
                                    curGh2[ii_h] = svH;
                                }
                            }
                        }
                    } catch (NumberFormatException e) {
                        continue;
                    }
                }
            }

            if (modelI >= 0 && curGh1 != null) {
                allGh1.add(new double[][]{curGh1});
                allGh2.add(new double[][]{curGh2});
                modelInfo.add(new int[]{curMax1, curMax2});
            }
        } catch (IOException e) {
            logger.error("Failed to open \"{}\" for reading.", mdfile);
            return 0;
        }

        model.nmodel = modelI + 1;

        for (int i = 0; i < model.nmodel; i++) {
            double[] rawGh1 = allGh1.get(i)[0];
            double[] rawGh2 = allGh2.get(i)[0];

            if (model.max2[i] == 0) {
                System.arraycopy(rawGh1, 0, model.gh1[i], 0, BFieldModel.MAXCOEFF);
                if (i + 1 < model.nmodel) {
                    double[] nextGh1 = allGh1.get(i + 1)[0];
                    System.arraycopy(nextGh1, 0, model.gh2[i], 0, BFieldModel.MAXCOEFF);
                }
            } else {
                System.arraycopy(rawGh1, 0, model.gh1[i], 0, BFieldModel.MAXCOEFF);
                System.arraycopy(rawGh2, 0, model.gh2[i], 0, BFieldModel.MAXCOEFF);
            }
        }

        if (model.nmodel > 0) {
            model.minyr = model.yrmin[0];
            model.maxyr = model.yrmax[0];
            for (int i = 1; i < model.nmodel; i++) {
                if (model.yrmin[i] < model.minyr) model.minyr = model.yrmin[i];
                if (model.yrmax[i] > model.maxyr) model.maxyr = model.yrmax[i];
            }
        }
        return 1;
    }

    /**
     * 解析 NOAA 官方发布的旧版 WMM.COF 头部。
     * 该格式只有三个字段（如 {@code 2025.0  WMM-2025  11/13/2024}），
     * 缺少内部格式中的阶数/有效期/高度范围，需按 WMM 规范补齐：
     * 阶数 12、有效期 5 年、高度 -1~600 km。
     * 数据行仍为 {@code n m g h dg dh}（nT 与 nT/yr），与内部格式一致。
     */
    private static boolean parseLegacyWmmHeader(BFieldModel model, int modelI, String[] parts) {
        if (parts.length < 2) return false;

        double epoch;
        try {
            epoch = Double.parseDouble(parts[0]);
        } catch (NumberFormatException e) {
            return false;
        }

        model.name[modelI] = parts[1];
        model.epoch[modelI] = epoch;
        model.max1[modelI] = WMM_DEGREE;
        model.max2[modelI] = WMM_DEGREE;
        model.max3[modelI] = 0;
        model.yrmin[modelI] = epoch;
        model.yrmax[modelI] = epoch + WMM_VALID_YEARS;
        model.altmin[modelI] = WMM_ALT_MIN_KM;
        model.altmax[modelI] = WMM_ALT_MAX_KM;
        return true;
    }

    private static int extrapsh(double date, double dte1, int nmax1, int nmax2, int gh,
                                double[] gh1, double[] gh2, double[] gha, double[] ghb) {
        int nmax;
        int k, l;
        int ii;
        double factor;

        if (!(gh == 3 || gh == 4)) {
            logger.error("extrapsh: fatal: gh may only be 3 or 4");
        }

        factor = date - dte1;
        if (nmax1 == nmax2) {
            k = nmax1 * (nmax1 + 2);
            nmax = nmax1;
        } else {
            if (nmax1 > nmax2) {
                k = nmax2 * (nmax2 + 2);
                l = nmax1 * (nmax1 + 2);
                if (gh == 3) {
                    for (ii = k + 1; ii <= l; ii++) {
                        if (ii < BFieldModel.MAXCOEFF) gha[ii] = gh1[ii] + factor * (-gh1[ii]);
                    }
                } else if (gh == 4) {
                    for (ii = k + 1; ii <= l; ii++) {
                        if (ii < BFieldModel.MAXCOEFF) ghb[ii] = gh1[ii] + factor * (-gh1[ii]);
                    }
                }
                nmax = nmax1;
            } else {
                k = nmax1 * (nmax1 + 2);
                l = nmax2 * (nmax2 + 2);
                if (gh == 3) {
                    for (ii = k + 1; ii <= l; ii++) {
                        if (ii < BFieldModel.MAXCOEFF) gha[ii] = factor * gh2[ii];
                    }
                } else if (gh == 4) {
                    for (ii = k + 1; ii <= l; ii++) {
                        if (ii < BFieldModel.MAXCOEFF) ghb[ii] = factor * gh2[ii];
                    }
                }
                nmax = nmax2;
            }
        }
        if (gh == 3) {
            for (ii = 1; ii <= k; ii++) {
                if (ii < BFieldModel.MAXCOEFF) gha[ii] = gh1[ii] + factor * gh2[ii];
            }
        } else if (gh == 4) {
            for (ii = 1; ii <= k; ii++) {
                if (ii < BFieldModel.MAXCOEFF) ghb[ii] = gh1[ii] + factor * gh2[ii];
            }
        }
        return nmax;
    }

    private static int interpsh(double date, double dte1, int nmax1, double dte2, int nmax2, int gh,
                                double[] gh1, double[] gh2, double[] gha, double[] ghb) {
        int nmax;
        int k, l;
        int ii;
        double factor;

        factor = (date - dte1) / (dte2 - dte1);
        if (nmax1 == nmax2) {
            k = nmax1 * (nmax1 + 2);
            nmax = nmax1;
        } else {
            if (nmax1 > nmax2) {
                k = nmax2 * (nmax2 + 2);
                l = nmax1 * (nmax1 + 2);
                if (gh == 3) {
                    for (ii = k + 1; ii <= l; ii++) {
                        if (ii < BFieldModel.MAXCOEFF) gha[ii] = gh1[ii] + factor * (-gh1[ii]);
                    }
                } else if (gh == 4) {
                    for (ii = k + 1; ii <= l; ii++) {
                        if (ii < BFieldModel.MAXCOEFF) ghb[ii] = gh1[ii] + factor * (-gh1[ii]);
                    }
                }
                nmax = nmax1;
            } else {
                k = nmax1 * (nmax1 + 2);
                l = nmax2 * (nmax2 + 2);
                if (gh == 3) {
                    for (ii = k + 1; ii <= l; ii++) {
                        if (ii < BFieldModel.MAXCOEFF) gha[ii] = factor * gh2[ii];
                    }
                } else if (gh == 4) {
                    for (ii = k + 1; ii <= l; ii++) {
                        if (ii < BFieldModel.MAXCOEFF) ghb[ii] = factor * gh2[ii];
                    }
                }
                nmax = nmax2;
            }
        }
        if (gh == 3) {
            for (ii = 1; ii <= k; ii++) {
                if (ii < BFieldModel.MAXCOEFF) gha[ii] = gh1[ii] + factor * (gh2[ii] - gh1[ii]);
            }
        } else if (gh == 4) {
            for (ii = 1; ii <= k; ii++) {
                if (ii < BFieldModel.MAXCOEFF) ghb[ii] = gh1[ii] + factor * (gh2[ii] - gh1[ii]);
            }
        }
        return nmax;
    }

    private static int shval3(int coordSys, double flat, double flon, double elev, int nmax,
                              int gh, int iext, double ext1, double ext2, double ext3,
                              double[] gha, double[] ghb, double[] xz, double[] yz, double[] zz) {
        double a2 = 40680631.59;
        double b2 = 40408299.98;
        double dtr = 0.01745329;
        double earthsRadius = EARTH_RADIUS;
        double slat, clat, ratio;
        double aa = 0.0, bb = 0.0, cc, dd;
        double rr = 0.0, fm = 0.0, fn = 0.0;
        double[] sl = new double[14];
        double[] cl = new double[14];
        double[] p = new double[119];
        double[] q = new double[119];
        int ios = 0;
        int k, l, m, n, npq;
        double argument, power;
        int ii, j;

        slat = Math.sin(flat * dtr);
        clat = Math.cos(flat * dtr);

        double aaLat;
        if ((90.0 - flat) < 0.001) {
            aaLat = 89.999;
        } else if ((90.0 + flat) < 0.001) {
            aaLat = -89.999;
        } else {
            aaLat = flat;
        }
        argument = aaLat * dtr;
        clat = Math.cos(argument);
        argument = flon * dtr;
        sl[1] = Math.sin(argument);
        cl[1] = Math.cos(argument);

        xz[0] = yz[0] = zz[0] = 0;

        double sd = 0.0;
        double cd = 1.0;
        double r = elev;
        l = 1;
        n = 0;
        m = 1;
        npq = (nmax * (nmax + 3)) / 2;
        if (coordSys == COORDSYS_GEODETIC) {
            aa = a2 * clat * clat;
            bb = b2 * slat * slat;
            cc = aa + bb;
            argument = cc;
            dd = Math.sqrt(argument);
            argument = elev * (elev + 2.0 * dd) + (a2 * aa + b2 * bb) / cc;
            r = Math.sqrt(argument);
            cd = (elev + dd) / r;
            sd = (a2 - b2) / dd * slat * clat / r;
            aa = slat;
            slat = slat * cd - clat * sd;
            clat = clat * cd + aa * sd;
        }
        ratio = earthsRadius / r;
        argument = 3.0;
        aa = Math.sqrt(argument);
        p[1] = 2.0 * slat;
        p[2] = 2.0 * clat;
        p[3] = 4.5 * slat * slat - 1.5;
        p[4] = 3.0 * aa * clat * slat;
        q[1] = -clat;
        q[2] = slat;
        q[3] = -3.0 * clat * slat;
        q[4] = aa * (slat * slat - clat * clat);

        for (k = 1; k <= npq; ++k) {
            if (n < m) {
                m = 0;
                n = n + 1;
                argument = ratio;
                power = n + 2;
                rr = Math.pow(argument, power);
                fn = n;
            }
            fm = m;
            if (k >= 5) {
                if (m == n) {
                    argument = (1.0 - 0.5 / fm);
                    aa = Math.sqrt(argument);
                    j = k - n - 1;
                    p[k] = (1.0 + 1.0 / fm) * aa * clat * p[j];
                    q[k] = aa * (clat * q[j] + slat / fm * p[j]);
                    sl[m] = sl[m - 1] * cl[1] + cl[m - 1] * sl[1];
                    cl[m] = cl[m - 1] * cl[1] - sl[m - 1] * sl[1];
                } else {
                    argument = fn * fn - fm * fm;
                    aa = Math.sqrt(argument);
                    argument = ((fn - 1.0) * (fn - 1.0)) - (fm * fm);
                    bb = Math.sqrt(argument) / aa;
                    cc = (2.0 * fn - 1.0) / aa;
                    ii = k - n;
                    j = k - 2 * n + 1;
                    p[k] = (fn + 1.0) * (cc * slat / fn * p[ii] - bb / (fn - 1.0) * p[j]);
                    q[k] = cc * (slat * q[ii] - clat / fn * p[ii]) - bb * q[j];
                }
            }
            switch (gh) {
            case 3:
                aa = rr * gha[l];
                break;
            case 4:
                aa = rr * ghb[l];
                break;
            }
            if (m == 0) {
                xz[0] += aa * q[k];
                zz[0] -= aa * p[k];
                l = l + 1;
            } else {
                switch (gh) {
                case 3:
                    bb = rr * gha[l + 1];
                    break;
                case 4:
                    bb = rr * ghb[l + 1];
                    break;
                }
                cc = aa * cl[m] + bb * sl[m];
                xz[0] = xz[0] + cc * q[k];
                zz[0] = zz[0] - cc * p[k];
                if (clat > 0) {
                    yz[0] += (aa * sl[m] - bb * cl[m]) * fm * p[k] / ((fn + 1.0) * clat);
                } else {
                    yz[0] += (aa * sl[m] - bb * cl[m]) * q[k] * slat;
                }
                l = l + 2;
            }
            m = m + 1;
        }
        if (iext != 0) {
            aa = ext2 * cl[1] + ext3 * sl[1];
            xz[0] = xz[0] - ext1 * clat + aa * slat;
            yz[0] = yz[0] + ext2 * sl[1] - ext3 * cl[1];
            zz[0] = zz[0] + ext1 * slat + aa * clat;
        }
        aa = xz[0];
        xz[0] = xz[0] * cd + zz[0] * sd;
        zz[0] = zz[0] * cd - aa * sd;
        return ios;
    }

    private static void dihf(double x, double y, double z, BField bfield) {
        double sn = 0.0001;
        bfield.h = Math.sqrt(x * x + y * y);
        bfield.f = Math.sqrt(x * x + y * y + z * z);
        if (bfield.f < sn) {
            bfield.d = Double.NaN;
            bfield.i = Double.NaN;
        } else {
            bfield.i = Math.atan2(z, bfield.h);
            if (bfield.h < sn) {
                bfield.d = Double.NaN;
            } else {
                double hpx = bfield.h + x;
                if (hpx < sn) {
                    bfield.d = PI;
                } else {
                    bfield.d = 2.0 * Math.atan2(y, hpx);
                }
            }
        }
    }

    private static void dihfCalc(double x, double y, double z, double[] d, double[] i, double[] h, double[] f) {
        double sn = 0.0001;
        h[0] = Math.sqrt(x * x + y * y);
        f[0] = Math.sqrt(x * x + y * y + z * z);
        if (f[0] < sn) {
            d[0] = Double.NaN;
            i[0] = Double.NaN;
        } else {
            i[0] = Math.atan2(z, h[0]);
            if (h[0] < sn) {
                d[0] = Double.NaN;
            } else {
                double hpx = h[0] + x;
                if (hpx < sn) {
                    d[0] = PI;
                } else {
                    d[0] = 2.0 * Math.atan2(y, hpx);
                }
            }
        }
    }
}