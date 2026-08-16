package org.gnss.ignav.contract;

import java.io.Serializable;

public class InsPrediction implements Serializable {

    private static final long serialVersionUID = 1L;

    private GTime time;
    private double[] posEcef;
    private double[] velEcef;
    private double[] attQuat;
    private double[] posCov;
    private double[] velCov;
    private double[] attCov;
    private double[] gyro;

    public InsPrediction() {
        this.time = new GTime();
        this.posEcef = new double[3];
        this.velEcef = new double[3];
        this.attQuat = new double[4];
        this.posCov = new double[9];
        this.velCov = new double[9];
        this.attCov = new double[9];
        this.gyro = new double[3];
    }

    public GTime getTime() {
        return time;
    }

    public void setTime(GTime time) {
        this.time = new GTime(time);
    }

    public double[] getPosEcef() {
        return posEcef;
    }

    public void setPosEcef(double[] posEcef) {
        this.posEcef = posEcef;
    }

    public double[] getVelEcef() {
        return velEcef;
    }

    public void setVelEcef(double[] velEcef) {
        this.velEcef = velEcef;
    }

    public double[] getAttQuat() {
        return attQuat;
    }

    public void setAttQuat(double[] attQuat) {
        this.attQuat = attQuat;
    }

    public double[] getPosCov() {
        return posCov;
    }

    public void setPosCov(double[] posCov) {
        this.posCov = posCov;
    }

    public double[] getVelCov() {
        return velCov;
    }

    public void setVelCov(double[] velCov) {
        this.velCov = velCov;
    }

    public double[] getAttCov() {
        return attCov;
    }

    public void setAttCov(double[] attCov) {
        this.attCov = attCov;
    }

    public double[] getGyro() {
        return gyro;
    }

    public void setGyro(double[] gyro) {
        this.gyro = gyro;
    }

    public double[] getAttDcm() {
        if (attQuat == null || attQuat.length < 4) return null;
        double q0 = attQuat[0], q1 = attQuat[1], q2 = attQuat[2], q3 = attQuat[3];
        double[] C = new double[9];
        C[0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
        C[1] = 2.0*(q1*q2 + q0*q3);
        C[2] = 2.0*(q1*q3 - q0*q2);
        C[3] = 2.0*(q1*q2 - q0*q3);
        C[4] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
        C[5] = 2.0*(q2*q3 + q0*q1);
        C[6] = 2.0*(q1*q3 + q0*q2);
        C[7] = 2.0*(q2*q3 - q0*q1);
        C[8] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        return C;
    }

    @Override
    public String toString() {
        return "InsPrediction{time=" + time + "}";
    }
}