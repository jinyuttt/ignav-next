package org.gnss.ignav.ins.aiding;

import org.gnss.ignav.ins.common.GeoMag;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.data.BField;
import org.gnss.ignav.ins.data.BFieldModel;
import org.gnss.ignav.ins.data.GTime;
import org.gnss.ignav.ins.data.Magd;
import org.gnss.ignav.ins.data.MagOpt;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.gnss.ignav.ins.mech.InsAlignMech;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class InsMagnetometer {

    private static final Logger logger = LoggerFactory.getLogger(InsMagnetometer.class);

    /** 磁航向量测噪声 1σ，物理量 5 deg，按弧度参与滤波计算 */
    private static final double VAR_MAG = 5.0 * IgnavConstants.D2R;

    private InsMagnetometer() {}

    public static int magmodel(String file) {
        logger.debug("magmodel, file={}", file);
        return GeoMag.magmodel(file);
    }

    /**
     * 由载体姿态把机体坐标系磁测值投影到水平面，消除俯仰/横滚的影响。
     *
     * @param data 磁测数据，{@code data.val} 为机体系三轴磁感应强度，单位 nT
     * @param Cbn  n 系（NED）到 b 系（机体）的旋转矩阵，无量纲
     * @param mh   输出，水平面内两正交轴的磁分量，单位 nT
     */
    private static void untilt(Magd data, double[] Cbn, double[] mh) {
        double r = -Math.atan(Cbn[2] / Math.sqrt(1.0 - InsMath.SQR(Cbn[2])));
        double p = Math.atan2(Cbn[5], Cbn[8]);
        double[] Cbh = new double[6];

        Cbh[0] = Math.cos(p);
        Cbh[2] = Math.sin(r) * Math.sin(p);
        Cbh[4] = Math.cos(r) * Math.sin(p);
        Cbh[1] = 0.0;
        Cbh[3] = Math.cos(r);
        Cbh[5] = -Math.sin(r);
        InsMath.matmul("NN", 2, 1, 3, 1.0, Cbh, data.val, 0.0, mh);
    }

    /**
     * 计算当前位置的磁偏角，用于把磁航向修正为真航向。
     *
     * @param pos 纬度、经度（rad）、椭球高（m）
     * @return 磁偏角，单位 deg（与 WMM 模型输出口径一致）
     */
    private static double undecli(InsState ins, double[] pos) {
        double[] ep = new double[6];
        GTime.time2epoch(ins.time, ep);
        BField mag = GeoMag.getMag();
        BFieldModel model = GeoMag.getMagModel();
        GeoMag.getFieldComponents(mag, model, pos[2], GeoMag.UNITS_METERS,
                GeoMag.COORDSYS_GEODETIC, pos[0] * IgnavConstants.R2D, pos[1] * IgnavConstants.R2D,
                GeoMag.julday((int) ep[1], (int) ep[2], (int) ep[0]));
        return mag.d;
    }

    /** 磁力计刻度因子与零偏改正，单位 nT */
    private static void undistortmag(MagOpt opt, double[] mh) {
        mh[0] = opt.sx * mh[0] + opt.ox;
        mh[1] = opt.sy * mh[1] + opt.oy;
    }

    /**
     * 由磁测值解算真航向。
     *
     * @return 真航向，单位 deg；磁航向为 atan2 结果的弧度换算成度后，再扣除磁偏角
     */
    public static double maghead(MagOpt opt, Magd data, double[] Cbn, double[] pos, InsState ins) {
        double[] mh = new double[2];
        double decli, yaw;

        logger.debug("magcalib:");

        untilt(data, Cbn, mh);
        undistortmag(opt, mh);

        decli = undecli(ins, pos);

        yaw = Math.atan2(mh[1], mh[0]) * IgnavConstants.R2D;
        return yaw - decli;
    }

    /**
     * 航向对姿态误差状态的雅可比。
     *
     * 结果写入 {@code dhdatt[0..2]}，对应状态向量中前三个姿态误差分量，
     * 其余状态（速度/位置/零偏）对航向无直接偏导，保持为 0。
     * 量纲为 ∂航向/∂姿态角，弧度比弧度，无量纲。
     */
    private static void jacobHead2att(double[] Cbe, double[] pos, double[] dhdatt) {
        double[] Cne = new double[9];
        double[] Cbn = new double[9];

        InsMath.ned2xyz(pos, Cne);
        InsMath.matmul("TN", 3, 3, 3, 1.0, Cne, Cbe, 0.0, Cbn);

        dhdatt[0] = (Cbn[0] * (Cne[5] * Cbe[1] - Cne[4] * Cbe[2]) - Cbn[1] * (Cne[2] * Cbe[1] - Cne[1] * Cbe[2])) /
                (InsMath.SQR(Cbn[0]) + InsMath.SQR(Cbn[1]));

        dhdatt[1] = (Cbn[0] * (-Cne[5] * Cbe[0] + Cne[3] * Cbe[2]) - Cbn[1] * (-Cne[2] * Cbe[1] + Cne[0] * Cbe[2])) /
                (InsMath.SQR(Cbn[0]) + InsMath.SQR(Cbn[1]));

        dhdatt[2] = (Cbn[0] * (Cne[4] * Cbe[0] - Cne[3] * Cbe[1]) - Cbn[1] * (Cne[1] * Cbe[0] - Cne[0] * Cbe[1])) /
                (InsMath.SQR(Cbn[0]) + InsMath.SQR(Cbn[1]));
    }

    /**
     * 磁航向的量测更新。
     *
     * 单位约定：{@code magh} 与 {@code yaw} 均为 deg；残差 {@code v} 换算为 rad 后
     * 与状态改正量保持同一量纲；{@code R} 为量测方差，rad²。
     */
    private static int magfilt(InsState ins, InsOpt opt, Magd data) {
        int nx = ins.nx;
        double[] H = new double[1 * nx];
        double[] x = ins.x;
        double[] P = ins.P;
        double R;
        double v;
        double[] Cbn = new double[9];
        double[] pos = new double[3];
        double[] Cne = new double[9];
        double magh, yaw;

        logger.debug("magfilt: nx={}", nx);

        InsMath.ecef2pos(ins.re, pos);
        InsMath.ned2xyz(pos, Cne);
        InsMath.matmul("TN", 3, 3, 3, 1.0, Cne, ins.Cbe, 0.0, Cbn);

        magh = maghead(opt.magopt, data, Cbn, pos, ins);
        yaw = Math.atan2(Cbn[1], Cbn[0]) * IgnavConstants.R2D;

        jacobHead2att(ins.Cbe, pos, H);

        v = normang(magh) - normang(yaw);
        v *= IgnavConstants.D2R;
        R = InsMath.SQR(VAR_MAG);

        double[] vArr = new double[]{v};
        double[] RArr = new double[]{R};

        if (InsAlignMech.filter(x, P, H, vArr, RArr, nx, 1) != 0) {
            logger.warn("filter error");
            return 0;
        }

        InsNhc.clp(ins, opt, x);
        return 1;
    }

    /**
     * 磁力计量测入口。
     *
     * @param data 磁测数据，{@code data.val} 为机体系三轴磁感应强度，单位 nT；
     *             模长为 0 视为无效观测
     * @return 1 表示更新成功，0 表示数据无效或滤波失败
     */
    public static int magnetometer(InsState ins, InsOpt opt, Magd data) {
        logger.debug("magnetometer:");

        if (InsMath.norm(data.val, 3) <= 0.0) {
            logger.warn("no valid magnetometer data");
            return 0;
        }
        if (magfilt(ins, opt, data) == 0) {
            logger.warn("magnetic head update fail");
            return 0;
        }
        ins.stat = IgnavConstants.INSS_MAGH;
        return 1;
    }

    private static double normang(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}