package org.gnss.ignav.contract;

import java.io.Serializable;

public class StateCorrection implements Serializable {

    private static final long serialVersionUID = 1L;

    private double[] dx;
    private double[] dP;

    public StateCorrection() {
        this.dx = null;
        this.dP = null;
    }

    public StateCorrection(double[] dx, double[] dP) {
        this.dx = dx;
        this.dP = dP;
    }

    public double[] getDx() {
        return dx;
    }

    public void setDx(double[] dx) {
        this.dx = dx;
    }

    public double[] getDP() {
        return dP;
    }

    public void setDP(double[] dP) {
        this.dP = dP;
    }

    @Override
    public String toString() {
        return "StateCorrection{dx=" + (dx != null ? dx.length : 0) +
               ", dP=" + (dP != null ? dP.length : 0) + "}";
    }
}