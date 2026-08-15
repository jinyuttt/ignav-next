package org.gnss.ignav.contract;

import java.io.Serializable;

public class GnssConfig implements Serializable {

    private static final long serialVersionUID = 1L;

    public enum FusionMode {
        LC(1),
        TC(2),
        STC(3);

        private final int code;

        FusionMode(int code) {
            this.code = code;
        }

        public int getCode() {
            return code;
        }

        public static FusionMode fromCode(int code) {
            for (FusionMode m : values()) {
                if (m.code == code) return m;
            }
            return LC;
        }
    }

    private FusionMode fusionMode;
    private double posMeasurementNoise;
    private double velMeasurementNoise;
    private double maxPositionInnovation;
    private double maxVelocityInnovation;
    private double maxSyncTimeDiff;
    private double chi2Alpha;
    private double maxUpdateTimeInterval;
    private int minSatellitesForTc;
    private boolean enableRaim;

    public GnssConfig() {
        this.fusionMode = FusionMode.LC;
        this.posMeasurementNoise = 2.5;
        this.velMeasurementNoise = 0.1;
        this.maxPositionInnovation = 1000.0;
        this.maxVelocityInnovation = 100.0;
        this.maxSyncTimeDiff = 1.0;
        this.chi2Alpha = 0.01;
        this.maxUpdateTimeInterval = 60.0;
        this.minSatellitesForTc = 5;
        this.enableRaim = false;
    }

    public FusionMode getFusionMode() { return fusionMode; }
    public void setFusionMode(FusionMode fusionMode) { this.fusionMode = fusionMode; }

    public double getPosMeasurementNoise() { return posMeasurementNoise; }
    public void setPosMeasurementNoise(double posMeasurementNoise) { this.posMeasurementNoise = posMeasurementNoise; }

    public double getVelMeasurementNoise() { return velMeasurementNoise; }
    public void setVelMeasurementNoise(double velMeasurementNoise) { this.velMeasurementNoise = velMeasurementNoise; }

    public double getMaxPositionInnovation() { return maxPositionInnovation; }
    public void setMaxPositionInnovation(double maxPositionInnovation) { this.maxPositionInnovation = maxPositionInnovation; }

    public double getMaxVelocityInnovation() { return maxVelocityInnovation; }
    public void setMaxVelocityInnovation(double maxVelocityInnovation) { this.maxVelocityInnovation = maxVelocityInnovation; }

    public double getMaxSyncTimeDiff() { return maxSyncTimeDiff; }
    public void setMaxSyncTimeDiff(double maxSyncTimeDiff) { this.maxSyncTimeDiff = maxSyncTimeDiff; }

    public double getChi2Alpha() { return chi2Alpha; }
    public void setChi2Alpha(double chi2Alpha) { this.chi2Alpha = chi2Alpha; }

    public double getMaxUpdateTimeInterval() { return maxUpdateTimeInterval; }
    public void setMaxUpdateTimeInterval(double maxUpdateTimeInterval) { this.maxUpdateTimeInterval = maxUpdateTimeInterval; }

    public int getMinSatellitesForTc() { return minSatellitesForTc; }
    public void setMinSatellitesForTc(int minSatellitesForTc) { this.minSatellitesForTc = minSatellitesForTc; }

    public boolean isEnableRaim() { return enableRaim; }
    public void setEnableRaim(boolean enableRaim) { this.enableRaim = enableRaim; }
}