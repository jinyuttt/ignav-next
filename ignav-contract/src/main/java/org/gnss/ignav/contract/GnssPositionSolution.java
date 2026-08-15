package org.gnss.ignav.contract;

import java.io.Serializable;

public class GnssPositionSolution implements Serializable {

    private static final long serialVersionUID = 1L;

    public enum SolutionStatus {
        NONE(0),
        FIX(1),
        FLOAT(2),
        SPP(3),
        DGPS(4),
        PPP(5),
        OTHER(9);

        private final int code;

        SolutionStatus(int code) {
            this.code = code;
        }

        public int getCode() {
            return code;
        }

        public static SolutionStatus fromCode(int code) {
            for (SolutionStatus s : values()) {
                if (s.code == code) return s;
            }
            return OTHER;
        }
    }

    private GTime time;
    private double[] posEcef;
    private double[] velEcef;
    private double[] posCov;
    private double[] velCov;
    private SolutionStatus solStatus;
    private int numSat;
    private float age;
    private float ratio;

    public GnssPositionSolution() {
        this.time = new GTime();
        this.posEcef = new double[3];
        this.velEcef = new double[3];
        this.posCov = new double[9];
        this.velCov = new double[9];
        this.solStatus = SolutionStatus.NONE;
        this.numSat = 0;
        this.age = 0.0f;
        this.ratio = 0.0f;
    }

    public GTime getTime() {
        return time;
    }

    public void setTime(GTime time) {
        this.time = new GTime(time);
    }

    public double[] getPosEcef() {
        return posEcef;
    }

    public void setPosEcef(double[] posEcef) {
        this.posEcef = posEcef;
    }

    public double[] getVelEcef() {
        return velEcef;
    }

    public void setVelEcef(double[] velEcef) {
        this.velEcef = velEcef;
    }

    public double[] getPosCov() {
        return posCov;
    }

    public void setPosCov(double[] posCov) {
        this.posCov = posCov;
    }

    public double[] getVelCov() {
        return velCov;
    }

    public void setVelCov(double[] velCov) {
        this.velCov = velCov;
    }

    public SolutionStatus getSolStatus() {
        return solStatus;
    }

    public void setSolStatus(SolutionStatus solStatus) {
        this.solStatus = solStatus;
    }

    public int getNumSat() {
        return numSat;
    }

    public void setNumSat(int numSat) {
        this.numSat = numSat;
    }

    public float getAge() {
        return age;
    }

    public void setAge(float age) {
        this.age = age;
    }

    public float getRatio() {
        return ratio;
    }

    public void setRatio(float ratio) {
        this.ratio = ratio;
    }

    public boolean isValid() {
        return solStatus == SolutionStatus.FIX ||
               solStatus == SolutionStatus.SPP ||
               solStatus == SolutionStatus.DGPS ||
               solStatus == SolutionStatus.PPP;
    }

    public boolean isFixed() {
        return solStatus == SolutionStatus.FIX;
    }

    @Override
    public String toString() {
        return "GnssPositionSolution{time=" + time + ", status=" + solStatus +
               ", numSat=" + numSat + "}";
    }
}