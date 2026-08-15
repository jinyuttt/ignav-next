package org.gnss.ignav.ins.data;

public class Odopt {

    public int dir;
    public int all;
    public double res;
    public double s;
    public double d;
    public double[] lever;
    public double odt;
    public double ostd;

    public Odopt() {
        this.dir = 0;
        this.all = 0;
        this.res = 0.0;
        this.s = 1.0;
        this.d = 0.0;
        this.lever = new double[3];
        this.odt = 0.0;
        this.ostd = 0.0;
    }
}
