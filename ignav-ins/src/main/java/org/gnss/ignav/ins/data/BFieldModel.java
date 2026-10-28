package org.gnss.ignav.ins.data;

public class BFieldModel {

    public static final int MAXMOD = 30;
    public static final int MAXDEG = 13;
    public static final int MAXCOEFF = MAXDEG * (MAXDEG + 2) + 1;

    public String[] name;
    public double[] epoch;
    public double[] yrmin;
    public double[] yrmax;
    public double minyr;
    public double maxyr;
    public double[] altmin;
    public double[] altmax;
    public int nmodel;
    public int[] max1;
    public int[] max2;
    public int[] max3;
    public long[] irecPos;
    public double[][] gh1;
    public double[][] gh2;

    public BFieldModel() {
        this.name = new String[MAXMOD];
        this.epoch = new double[MAXMOD];
        this.yrmin = new double[MAXMOD];
        this.yrmax = new double[MAXMOD];
        this.minyr = 0.0;
        this.maxyr = 0.0;
        this.altmin = new double[MAXMOD];
        this.altmax = new double[MAXMOD];
        this.nmodel = 0;
        this.max1 = new int[MAXMOD];
        this.max2 = new int[MAXMOD];
        this.max3 = new int[MAXMOD];
        this.irecPos = new long[MAXMOD];
        this.gh1 = new double[MAXMOD][MAXCOEFF];
        this.gh2 = new double[MAXMOD][MAXCOEFF];
    }
}