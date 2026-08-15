package org.gnss.ignav.ins.data;

public class InsState {

    public GTime time;
    public GTime ptime;
    public GTime plct;
    public GTime ptct;
    public double dt;

    public double[] Cbe;
    public double[] re;
    public double[] ve;
    public double[] ae;
    public double[] dtr;
    public double dtrr;

    public double[] rn;
    public double[] vn;
    public double[] an;
    public double[] Cbn;
    public double[] dvn;

    public double[] ba;
    public double[] bg;
    public double[] Ma;
    public double[] Mg;
    public double[] Gg;
    public double[] fb0;
    public double[] omgb0;
    public double[] fb;
    public double[] omgb;

    public double[] fbp;
    public double[] omgbp;
    public double[] lever;
    public double[] lbc;

    public double[] Cbr;
    public double os;
    public double[] rbl;
    public double[] vr;

    public double[] Cvb;
    public double len;

    public double[] dopv;

    public int nx;
    public int nb;

    public double[] x;
    public double[] P;
    public double[] xa;
    public double[] Pa;
    public double[] xb;
    public double[] Pb;
    public double[] P0;
    public double[] F;

    public double[] pins;
    public double[] pCbe;
    public Gmeas gmeas;

    public double age;
    public double ratio;
    public int stat;
    public int gstat;
    public int pose;
    public int ns;

    public InsState() {
        this.time = new GTime();
        this.ptime = new GTime();
        this.plct = new GTime();
        this.ptct = new GTime();
        this.dt = 0.0;

        this.Cbe = new double[9];
        this.re = new double[3];
        this.ve = new double[3];
        this.ae = new double[3];
        this.dtr = new double[6];
        this.dtrr = 0.0;

        this.rn = new double[3];
        this.vn = new double[3];
        this.an = new double[3];
        this.Cbn = new double[9];
        this.dvn = new double[3];

        this.ba = new double[3];
        this.bg = new double[3];
        this.Ma = new double[9];
        this.Mg = new double[9];
        this.Gg = new double[9];
        this.fb0 = new double[3];
        this.omgb0 = new double[3];
        this.fb = new double[3];
        this.omgb = new double[3];

        this.fbp = new double[3];
        this.omgbp = new double[3];
        this.lever = new double[3];
        this.lbc = new double[3];

        this.Cbr = new double[9];
        this.os = 0.0;
        this.rbl = new double[3];
        this.vr = new double[3];

        this.Cvb = new double[9];
        this.len = 0.0;

        this.dopv = new double[3];

        this.nx = 0;
        this.nb = 0;

        this.x = null;
        this.P = null;
        this.xa = null;
        this.Pa = null;
        this.xb = null;
        this.Pb = null;
        this.P0 = null;
        this.F = null;

        this.pins = new double[9];
        this.pCbe = new double[9];
        this.gmeas = new Gmeas();

        this.age = 0.0;
        this.ratio = 0.0;
        this.stat = 0;
        this.gstat = 0;
        this.pose = 0;
        this.ns = 0;
    }
}