package org.gnss.ignav.ins.data;

import org.gnss.ignav.ins.common.IgnavConstants;

public class InsAlign {

    public double[] eb;
    public double[] db;
    public double[] web;
    public double[] wdb;
    public int ns;
    public int chkstatic;
    public double dt;
    public double[] phi0;
    public double[] wvn;

    public double[] pos;
    public double[] att;
    public double[] vel;
    public int method;
    public double pitchThres;
    public double velThres;
    public double gyroThres;
    public int nPos;
    public int nVel;
    public int nAtt;
    public double posVar;
    public double velVar;
    public double attVar;
    public double tInit;

    public InsAlign() {
        this.eb = new double[3];
        this.db = new double[3];
        this.web = new double[3];
        this.wdb = new double[3];
        this.ns = 0;
        this.chkstatic = 0;
        this.dt = 0.0;
        this.phi0 = new double[3];
        this.wvn = new double[3];

        this.pos = new double[3];
        this.att = new double[3];
        this.vel = new double[3];
        this.method = IgnavConstants.INS_ALIGN_COARSE;
        this.pitchThres = 0.0;
        this.velThres = 0.0;
        this.gyroThres = 0.0;
        this.nPos = 0;
        this.nVel = 0;
        this.nAtt = 0;
        this.posVar = 0.0;
        this.velVar = 0.0;
        this.attVar = 0.0;
        this.tInit = 0.0;
    }
}