package org.gnss.ignav.ins.data;

public class InsOpt {

    public int exinserr;
    public int inithead;
    public int gravityex;
    public int updint;
    public int baopt;
    public int bgopt;
    public int cnscl;
    public int estsg;
    public int estsa;
    public int estdt;
    public int estrg;
    public int estra;
    public int estlever;

    public int odo;
    public int poseAid;

    public int estodos;
    public int estodoa;
    public int estodol;
    public int estmisv;

    public int baproopt;
    public int bgproopt;
    public int sgproopt;
    public int saproopt;
    public int dtproopt;
    public int rgproopt;
    public int raproopt;
    public int osproopt;
    public int olproopt;
    public int oaproopt;
    public int cmaopt;
    public int vmaopt;

    public int alignVn;
    public int alignFn;
    public int alignCorse;
    public int alimethod;
    public int alignDualants;

    public int imuformat;
    public int imudecfmt;
    public int imucoors;
    public int imuvalfmt;
    public int lcopt;

    public int exprn;
    public int exphi;
    public int exvm;
    public int iisu;

    public int nhc;
    public int zvu;
    public int zaru;
    public int detst;
    public int magh;

    public int tc;
    public int lc;
    public int dopp;
    public int usecam;
    public int intpref;
    public int minp;
    public int soltype;
    public int transmitCorr;

    public GTime[][] ext;

    public double[] lever;
    public double[] misEuler;
    public double len;
    public double hz;
    public double nhz;

    public InsPsd psd;
    public InsUnc unc;
    public ImuErr imuerr;
    public InsAlign align;
    public InsZvOpt zvopt;
    public Odopt odopt;
    public MagOpt magopt;

    public InsOpt() {
        this.ext = new GTime[16][2];
        for (int i = 0; i < 16; i++) {
            ext[i][0] = new GTime();
            ext[i][1] = new GTime();
        }
        this.lever = new double[3];
        this.misEuler = new double[3];
        this.len = 0.0;
        this.hz = 0.0;
        this.nhz = 0.0;
        this.psd = new InsPsd();
        this.unc = new InsUnc();
        this.imuerr = new ImuErr();
        this.align = new InsAlign();
        this.zvopt = new InsZvOpt();
        this.odopt = new Odopt();
        this.magopt = new MagOpt();
    }
}