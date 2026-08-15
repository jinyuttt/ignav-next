package org.gnss.ignav.contract;

import java.io.Serializable;

public class ImuMeasurement implements Serializable {

    private static final long serialVersionUID = 1L;

    private GTime time;
    private double[] gyro;
    private double[] accl;

    public ImuMeasurement() {
        this.time = new GTime();
        this.gyro = new double[3];
        this.accl = new double[3];
    }

    public ImuMeasurement(GTime time, double[] gyro, double[] accl) {
        this.time = new GTime(time);
        this.gyro = gyro.clone();
        this.accl = accl.clone();
    }

    public GTime getTime() {
        return time;
    }

    public void setTime(GTime time) {
        this.time = new GTime(time);
    }

    public double[] getGyro() {
        return gyro;
    }

    public void setGyro(double[] gyro) {
        this.gyro = gyro.clone();
    }

    public double[] getAccl() {
        return accl;
    }

    public void setAccl(double[] accl) {
        this.accl = accl.clone();
    }

    @Override
    public String toString() {
        return "ImuMeasurement{time=" + time + "}";
    }
}