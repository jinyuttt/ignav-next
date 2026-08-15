package org.gnss.ignav.ins.data;

public class Imud {

    public GTime time;
    public double[] gyro;
    public double[] accl;
    public double temp;
    public int stat;
    public int pps;
    public int imuc;
    public short odoc;
    public Odod odo;

    public Imud() {
        this.time = new GTime();
        this.gyro = new double[3];
        this.accl = new double[3];
        this.temp = 0.0;
        this.stat = 0;
        this.pps = 0;
        this.imuc = 0;
        this.odoc = 0;
        this.odo = new Odod();
    }

    public Imud(Imud other) {
        this.time = new GTime(other.time);
        this.gyro = other.gyro.clone();
        this.accl = other.accl.clone();
        this.temp = other.temp;
        this.stat = other.stat;
        this.pps = other.pps;
        this.imuc = other.imuc;
        this.odoc = other.odoc;
        this.odo = new Odod(other.odo);
    }
}