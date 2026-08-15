package org.gnss.ignav.fusion;

import org.gnss.ignav.contract.FusionMode;
import org.gnss.ignav.contract.GnssConfig;
import org.gnss.ignav.contract.InsConfig;

import java.io.Serializable;

public class FusionConfig implements Serializable {

    private static final long serialVersionUID = 1L;

    private InsConfig insConfig;
    private GnssConfig gnssConfig;
    private FusionMode fusionMode;
    private double maxInsCovForReset;
    private double maxGnssAgeForLc;
    private double maxGnssAgeForStc;
    private double maxGnssAgeForTc;
    private boolean enableAdaptiveMode;
    private boolean enableFeedbackCorrection;
    private boolean enableSmoothing;
    private double chi2Threshold;
    private int stateDimension;

    public FusionConfig() {
        this.insConfig = new InsConfig();
        this.gnssConfig = new GnssConfig();
        this.fusionMode = new FusionMode();
        this.maxInsCovForReset = 1e8;
        this.maxGnssAgeForLc = 30.0;
        this.maxGnssAgeForStc = 10.0;
        this.maxGnssAgeForTc = 5.0;
        this.enableAdaptiveMode = true;
        this.enableFeedbackCorrection = true;
        this.enableSmoothing = false;
        this.chi2Threshold = 0.01;
        this.stateDimension = 15;
    }

    public InsConfig getInsConfig() { return insConfig; }
    public void setInsConfig(InsConfig insConfig) { this.insConfig = insConfig; }

    public GnssConfig getGnssConfig() { return gnssConfig; }
    public void setGnssConfig(GnssConfig gnssConfig) { this.gnssConfig = gnssConfig; }

    public FusionMode getFusionMode() { return fusionMode; }
    public void setFusionMode(FusionMode fusionMode) { this.fusionMode = fusionMode; }

    public double getMaxInsCovForReset() { return maxInsCovForReset; }
    public void setMaxInsCovForReset(double maxInsCovForReset) { this.maxInsCovForReset = maxInsCovForReset; }

    public double getMaxGnssAgeForLc() { return maxGnssAgeForLc; }
    public void setMaxGnssAgeForLc(double maxGnssAgeForLc) { this.maxGnssAgeForLc = maxGnssAgeForLc; }

    public double getMaxGnssAgeForStc() { return maxGnssAgeForStc; }
    public void setMaxGnssAgeForStc(double maxGnssAgeForStc) { this.maxGnssAgeForStc = maxGnssAgeForStc; }

    public double getMaxGnssAgeForTc() { return maxGnssAgeForTc; }
    public void setMaxGnssAgeForTc(double maxGnssAgeForTc) { this.maxGnssAgeForTc = maxGnssAgeForTc; }

    public boolean isEnableAdaptiveMode() { return enableAdaptiveMode; }
    public void setEnableAdaptiveMode(boolean enableAdaptiveMode) { this.enableAdaptiveMode = enableAdaptiveMode; }

    public boolean isEnableFeedbackCorrection() { return enableFeedbackCorrection; }
    public void setEnableFeedbackCorrection(boolean enableFeedbackCorrection) { this.enableFeedbackCorrection = enableFeedbackCorrection; }

    public boolean isEnableSmoothing() { return enableSmoothing; }
    public void setEnableSmoothing(boolean enableSmoothing) { this.enableSmoothing = enableSmoothing; }

    public double getChi2Threshold() { return chi2Threshold; }
    public void setChi2Threshold(double chi2Threshold) { this.chi2Threshold = chi2Threshold; }

    public int getStateDimension() { return stateDimension; }
    public void setStateDimension(int stateDimension) { this.stateDimension = stateDimension; }

    @Override
    public String toString() {
        return "FusionConfig{mode=" + fusionMode.getMode() +
               ", adaptive=" + enableAdaptiveMode +
               ", feedback=" + enableFeedbackCorrection +
               ", smoothing=" + enableSmoothing +
               ", nx=" + stateDimension + "}";
    }
}