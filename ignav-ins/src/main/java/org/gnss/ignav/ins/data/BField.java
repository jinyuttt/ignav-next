package org.gnss.ignav.ins.data;

public class BField {

    public double d;
    public double i;
    public double h;
    public double f;
    public double x;
    public double y;
    public double z;
    public double ddot;
    public double fdot;
    public double hdot;
    public double idot;
    public double xdot;
    public double ydot;
    public double zdot;

    public BField() {
        this.d = 0.0;
        this.i = 0.0;
        this.h = 0.0;
        this.f = 0.0;
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
        this.ddot = 0.0;
        this.fdot = 0.0;
        this.hdot = 0.0;
        this.idot = 0.0;
        this.xdot = 0.0;
        this.ydot = 0.0;
        this.zdot = 0.0;
    }
}