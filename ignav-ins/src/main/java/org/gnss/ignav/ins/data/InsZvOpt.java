package org.gnss.ignav.ins.data;

public class InsZvOpt {

    public int ws;
    public double mt;
    public double sp;
    public double gthres;
    public double[] athres;
    public double[] gyrothres;
    public double odt;
    public double sigA;
    public double sigG;
    public double[] gamma;

    public InsZvOpt() {
        this.ws = 0;
        this.mt = 0.0;
        this.sp = 0.0;
        this.gthres = 0.0;
        this.athres = new double[3];
        this.gyrothres = new double[3];
        this.odt = 0.0;
        this.sigA = 0.0;
        this.sigG = 0.0;
        this.gamma = new double[4];
    }
}
