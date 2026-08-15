package org.gnss.ignav.contract;

import java.io.Serializable;

public class InsSolution implements Serializable {

    private static final long serialVersionUID = 1L;

    private GTime time;
    private double[] posEcef;
    private double[] velEcef;
    private double[] attQuat;
    private double[] posCov;
    private double[] velCov;
    private double[] attCov;
    private int status;

    public static final int STATUS_NONE = 0;
    public static final int STATUS_ALIGNING = 1;
    public static final int STATUS_ALIGNED = 2;
    public static final int STATUS_NAVIGATING = 3;

    public InsSolution() {
        this.time = new GTime();
        this.posEcef = new double[3];
        this.velEcef = new double[3];
        this.attQuat = new double[4];
        this.posCov = new double[9];
        this.velCov = new double[9];
        this.attCov = new double[9];
        this.status = STATUS_NONE;
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

    public int getStatus() {
        return status;
    }

    public void setStatus(int status) {
        this.status = status;
    }

    @Override
    public String toString() {
        return "InsSolution{time=" + time + ", status=" + status + "}";
    }
}