package org.gnss.ignav.ins.data;

public class Magd {

    public GTime time;
    public double[] val;

    public Magd() {
        this.time = new GTime();
        this.val = new double[3];
    }

    public Magd(Magd other) {
        this.time = new GTime(other.time);
        this.val = new double[3];
        System.arraycopy(other.val, 0, this.val, 0, 3);
    }
}