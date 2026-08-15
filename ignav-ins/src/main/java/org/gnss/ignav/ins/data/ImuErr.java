package org.gnss.ignav.ins.data;

public class ImuErr {

    public double[] bg;
    public double[] ba;
    public double[] Ma;
    public double[] Mg;
    public double[] Gg;
    public double TauG;
    public double TauA;
    public double[] wbg;

    public ImuErr() {
        this.bg = new double[3];
        this.ba = new double[3];
        this.Ma = new double[9];
        this.Mg = new double[9];
        this.Gg = new double[9];
        this.TauG = 0.0;
        this.TauA = 0.0;
        this.wbg = new double[3];
    }
}