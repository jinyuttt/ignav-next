package org.gnss.ignav.contract;

import java.io.Serializable;

public class GnssObservation implements Serializable {

    private static final long serialVersionUID = 1L;

    private GTime time;
    private double[] v;
    private double[] H;
    private double[] R;
    private int nm;
    private int nx;

    public GnssObservation() {
        this.time = new GTime();
        this.v = null;
        this.H = null;
        this.R = null;
        this.nm = 0;
        this.nx = 0;
    }

    public GnssObservation(GTime time, double[] v, double[] H, double[] R, int nm, int nx) {
        this.time = new GTime(time);
        this.v = v;
        this.H = H;
        this.R = R;
        this.nm = nm;
        this.nx = nx;
    }

    public GTime getTime() {
        return time;
    }

    public void setTime(GTime time) {
        this.time = new GTime(time);
    }

    public double[] getV() {
        return v;
    }

    public void setV(double[] v) {
        this.v = v;
    }

    public double[] getH() {
        return H;
    }

    public void setH(double[] H) {
        this.H = H;
    }

    public double[] getR() {
        return R;
    }

    public void setR(double[] R) {
        this.R = R;
    }

    public int getNm() {
        return nm;
    }

    public void setNm(int nm) {
        this.nm = nm;
    }

    public int getNx() {
        return nx;
    }

    public void setNx(int nx) {
        this.nx = nx;
    }

    @Override
    public String toString() {
        return "GnssObservation{time=" + time + ", nm=" + nm + ", nx=" + nx + "}";
    }
}