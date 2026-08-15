package org.gnss.ignav.contract;

import java.io.Serializable;

public class FusionMode implements Serializable {

    private static final long serialVersionUID = 1L;

    public enum Mode {
        INS_ONLY,
        LC,
        STC,
        TC
    }

    private Mode mode;
    private int minSatsTC;
    private int minSatsSTC;
    private int minSatsLC;
    private double maxGdopTC;
    private double maxGdopSTC;
    private double maxGdopLC;
    private int hysteresisEpochs;
    private int cooldownEpochs;
    private int epochsInCurrentMode;
    private int epochsSinceLastSwitch;

    public FusionMode() {
        this.mode = Mode.LC;
        this.minSatsTC = 6;
        this.minSatsSTC = 4;
        this.minSatsLC = 2;
        this.maxGdopTC = 3.0;
        this.maxGdopSTC = 6.0;
        this.maxGdopLC = 20.0;
        this.hysteresisEpochs = 10;
        this.cooldownEpochs = 30;
        this.epochsInCurrentMode = 0;
        this.epochsSinceLastSwitch = 0;
    }

    public Mode determineMode(int numSat, double gdop) {
        if (epochsSinceLastSwitch < cooldownEpochs && epochsSinceLastSwitch > 0) {
            epochsInCurrentMode++;
            epochsSinceLastSwitch++;
            return mode;
        }

        Mode newMode;
        if (numSat >= minSatsTC && gdop < maxGdopTC) {
            newMode = Mode.TC;
        } else if (numSat >= minSatsSTC && gdop < maxGdopSTC) {
            newMode = Mode.STC;
        } else if (numSat >= minSatsLC && gdop < maxGdopLC) {
            newMode = Mode.LC;
        } else {
            newMode = Mode.INS_ONLY;
        }

        if (newMode != mode) {
            if (epochsInCurrentMode >= hysteresisEpochs) {
                mode = newMode;
                epochsInCurrentMode = 0;
                epochsSinceLastSwitch = 0;
            }
        } else {
            epochsInCurrentMode++;
        }

        epochsSinceLastSwitch++;
        return mode;
    }

    public Mode getMode() { return mode; }
    public void setMode(Mode mode) {
        this.mode = mode;
        this.epochsInCurrentMode = 0;
        this.epochsSinceLastSwitch = 0;
    }

    public int getMinSatsTC() { return minSatsTC; }
    public void setMinSatsTC(int v) { this.minSatsTC = v; }
    public int getMinSatsSTC() { return minSatsSTC; }
    public void setMinSatsSTC(int v) { this.minSatsSTC = v; }
    public int getMinSatsLC() { return minSatsLC; }
    public void setMinSatsLC(int v) { this.minSatsLC = v; }
    public double getMaxGdopTC() { return maxGdopTC; }
    public void setMaxGdopTC(double v) { this.maxGdopTC = v; }
    public double getMaxGdopSTC() { return maxGdopSTC; }
    public void setMaxGdopSTC(double v) { this.maxGdopSTC = v; }
    public double getMaxGdopLC() { return maxGdopLC; }
    public void setMaxGdopLC(double v) { this.maxGdopLC = v; }
    public int getHysteresisEpochs() { return hysteresisEpochs; }
    public void setHysteresisEpochs(int v) { this.hysteresisEpochs = v; }
    public int getCooldownEpochs() { return cooldownEpochs; }
    public void setCooldownEpochs(int v) { this.cooldownEpochs = v; }

    @Override
    public String toString() {
        return "FusionMode{mode=" + mode + ", epochsInMode=" + epochsInCurrentMode + "}";
    }
}