package org.gnss.ignav.ins.data;

public class Gmea {

    public GTime t;
    public int ns;
    public int stat;
    public double[] pe;
    public double[] ve;
    public double[] std;
    public double[] covp;
    public double[] covv;

    public Gmea() {
        this.t = new GTime();
        this.ns = 0;
        this.stat = 0;
        this.pe = new double[3];
        this.ve = new double[3];
        this.std = new double[6];
        this.covp = new double[9];
        this.covv = new double[9];
    }

    public Gmea(Gmea other) {
        this.t = new GTime(other.t);
        this.ns = other.ns;
        this.stat = other.stat;
        this.pe = other.pe.clone();
        this.ve = other.ve.clone();
        this.std = other.std.clone();
        this.covp = other.covp.clone();
        this.covv = other.covv.clone();
    }

    public static void copy(Gmea src, Gmea dst) {
        dst.t = new GTime(src.t);
        dst.ns = src.ns;
        dst.stat = src.stat;
        System.arraycopy(src.pe, 0, dst.pe, 0, 3);
        System.arraycopy(src.ve, 0, dst.ve, 0, 3);
        System.arraycopy(src.std, 0, dst.std, 0, 6);
        System.arraycopy(src.covp, 0, dst.covp, 0, 9);
        System.arraycopy(src.covv, 0, dst.covv, 0, 9);
    }
}
