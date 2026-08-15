package org.gnss.ignav.ins.common;

public final class Quaternion {

    public double w;
    public double x;
    public double y;
    public double z;

    public Quaternion() {
        this.w = 1.0;
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
    }

    public Quaternion(double w, double x, double y, double z) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static final Quaternion IDENTITY = new Quaternion(1.0, 0.0, 0.0, 0.0);

    private static final double ZERO_TOLERANCE = 1e-6;

    public static Quaternion copy(Quaternion qi) {
        return new Quaternion(qi.w, qi.x, qi.y, qi.z);
    }

    public void copyFrom(Quaternion qi) {
        this.w = qi.w;
        this.x = qi.x;
        this.y = qi.y;
        this.z = qi.z;
    }

    public double len() {
        return Math.sqrt(w * w + x * x + y * y + z * z);
    }

    public static double len(Quaternion q) {
        return Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    }

    public void normalizeSelf() {
        double n = len();
        if (n > 0.0) {
            w /= n;
            x /= n;
            y /= n;
            z /= n;
        }
    }

    public static Quaternion normalize(Quaternion qi) {
        Quaternion qo = copy(qi);
        qo.normalizeSelf();
        return qo;
    }

    public static void normalize(Quaternion qo, Quaternion qi) {
        double n = len(qi);
        if (n > 0.0) {
            qo.w = qi.w / n;
            qo.x = qi.x / n;
            qo.y = qi.y / n;
            qo.z = qi.z / n;
        } else {
            qo.copyFrom(qi);
        }
    }

    public static Quaternion conj(Quaternion qi) {
        return new Quaternion(qi.w, -qi.x, -qi.y, -qi.z);
    }

    public static void conj(Quaternion qo, Quaternion qi) {
        qo.w = qi.w;
        qo.x = -qi.x;
        qo.y = -qi.y;
        qo.z = -qi.z;
    }

    public static Quaternion inv(Quaternion qi) {
        return new Quaternion(qi.w, -qi.x, -qi.y, -qi.z);
    }

    public static void mul(Quaternion o, Quaternion q1, Quaternion q2) {
        o.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
        o.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
        o.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
        o.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    }

    public static void toRhRotMatrix(Quaternion q, double[] m) {
        InsMath.quatToRhRotMatrix(new double[]{q.w, q.x, q.y, q.z}, m);
    }

    public static void fromDcm(double[] C, Quaternion q) {
        double[] qa = new double[4];
        InsMath.dcm2quat(C, qa);
        q.w = qa[0];
        q.x = qa[1];
        q.y = qa[2];
        q.z = qa[3];
    }

    public static void toDcm(Quaternion q, double[] C) {
        InsMath.quat2dcm(new double[]{q.w, q.x, q.y, q.z}, C);
    }

    public static void toEuler(double[] euler, Quaternion q) {
        double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z, ww = q.w * q.w;
        euler[2] = Math.atan2(2.0 * (q.x * q.y + q.z * q.w), xx - yy - zz + ww);
        euler[1] = Math.asin(-2.0 * (q.x * q.z - q.y * q.w));
        euler[0] = Math.atan2(2.0 * (q.y * q.z + q.x * q.w), -xx - yy + zz + ww);
    }

    public static double normalizeEuler02Pi(double ang) {
        while (ang < 0.0) ang += 2.0 * Math.PI;
        while (ang >= 2.0 * Math.PI) ang -= 2.0 * Math.PI;
        return ang;
    }

    public static void rotVec(double[] vo, double[] vi, Quaternion q) {
        double vx = vi[0], vy = vi[1], vz = vi[2];
        double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
        double qww = qw * qw, qxx = qx * qx, qyy = qy * qy, qzz = qz * qz;
        double qwx = qw * qx, qwy = qw * qy, qwz = qw * qz;
        double qxy = qx * qy, qxz = qx * qz, qyz = qy * qz;
        vo[0] = (qww + qxx - qyy - qzz) * vx + 2.0 * ((qxy - qwz) * vy + (qxz + qwy) * vz);
        vo[1] = (qww - qxx + qyy - qzz) * vy + 2.0 * ((qxy + qwz) * vx + (qyz - qwx) * vz);
        vo[2] = (qww - qxx - qyy + qzz) * vz + 2.0 * ((qxz - qwy) * vx + (qyz + qwx) * vy);
    }
}