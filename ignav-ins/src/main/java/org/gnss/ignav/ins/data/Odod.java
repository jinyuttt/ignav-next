package org.gnss.ignav.ins.data;

public class Odod {

    public GTime time;
    public double dt;
    public double dr;
    public double[] vr;

    public Odod() {
        this.time = new GTime();
        this.dt = 0.0;
        this.dr = 0.0;
        this.vr = new double[3];
    }

    public Odod(Odod other) {
        this.time = new GTime(other.time);
        this.dt = other.dt;
        this.dr = other.dr;
        this.vr = other.vr.clone();
    }
}