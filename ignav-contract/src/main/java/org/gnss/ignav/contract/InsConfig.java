package org.gnss.ignav.contract;

import java.io.Serializable;

public class InsConfig implements Serializable {

    private static final long serialVersionUID = 1L;

    private double[] initPosEcef;
    private double[] initVelEcef;
    private double[] initAttQuat;
    private double[] initPosStd;
    private double[] initVelStd;
    private double initAttStd;
    private double gyroNoisePsd;
    private double acclNoisePsd;
    private double bgPsd;
    private double baPsd;
    private double dtPsd;
    private double sgPsd;
    private double saPsd;
    private double rgPsd;
    private double raPsd;
    private double osPsd;
    private double olPsd;
    private double oaPsd;
    private double clkPsd;
    private double clkrPsd;
    private boolean estimateBa;
    private boolean estimateBg;
    private boolean estimateDt;
    private boolean estimateSg;
    private boolean estimateSa;
    private boolean estimateRg;
    private boolean estimateRa;
    private boolean estimateLever;
    private boolean estimateOdo;
    private boolean estimateMagnetometer;
    private boolean enableNhc;
    private boolean enableZvu;
    private boolean enableZaru;
    private boolean enableOdo;
    private boolean enableMagnetometer;
    private double[] leverArm;
    private double odoScale;
    private double[] odoLever;
    private double odoAngle;

    public InsConfig() {
        this.initPosEcef = new double[3];
        this.initVelEcef = new double[3];
        this.initAttQuat = new double[4];
        this.initPosStd = new double[3];
        this.initVelStd = new double[3];
        this.initAttStd = 0.0;
        this.gyroNoisePsd = 0.0;
        this.acclNoisePsd = 0.0;
        this.bgPsd = 0.0;
        this.baPsd = 0.0;
        this.dtPsd = 0.0;
        this.sgPsd = 0.0;
        this.saPsd = 0.0;
        this.rgPsd = 0.0;
        this.raPsd = 0.0;
        this.osPsd = 0.0;
        this.olPsd = 0.0;
        this.oaPsd = 0.0;
        this.clkPsd = 0.0;
        this.clkrPsd = 0.0;
        this.estimateBa = false;
        this.estimateBg = false;
        this.estimateDt = false;
        this.estimateSg = false;
        this.estimateSa = false;
        this.estimateRg = false;
        this.estimateRa = false;
        this.estimateLever = false;
        this.estimateOdo = false;
        this.estimateMagnetometer = false;
        this.enableNhc = false;
        this.enableZvu = false;
        this.enableZaru = false;
        this.enableOdo = false;
        this.enableMagnetometer = false;
        this.leverArm = new double[3];
        this.odoScale = 0.0;
        this.odoLever = new double[3];
        this.odoAngle = 0.0;
    }

    public double[] getInitPosEcef() { return initPosEcef; }
    public void setInitPosEcef(double[] initPosEcef) { this.initPosEcef = initPosEcef; }

    public double[] getInitVelEcef() { return initVelEcef; }
    public void setInitVelEcef(double[] initVelEcef) { this.initVelEcef = initVelEcef; }

    public double[] getInitAttQuat() { return initAttQuat; }
    public void setInitAttQuat(double[] initAttQuat) { this.initAttQuat = initAttQuat; }

    public double[] getInitPosStd() { return initPosStd; }
    public void setInitPosStd(double[] initPosStd) { this.initPosStd = initPosStd; }

    public double[] getInitVelStd() { return initVelStd; }
    public void setInitVelStd(double[] initVelStd) { this.initVelStd = initVelStd; }

    public double getInitAttStd() { return initAttStd; }
    public void setInitAttStd(double initAttStd) { this.initAttStd = initAttStd; }

    public double getGyroNoisePsd() { return gyroNoisePsd; }
    public void setGyroNoisePsd(double gyroNoisePsd) { this.gyroNoisePsd = gyroNoisePsd; }

    public double getAcclNoisePsd() { return acclNoisePsd; }
    public void setAcclNoisePsd(double acclNoisePsd) { this.acclNoisePsd = acclNoisePsd; }

    public double getBgPsd() { return bgPsd; }
    public void setBgPsd(double bgPsd) { this.bgPsd = bgPsd; }

    public double getBaPsd() { return baPsd; }
    public void setBaPsd(double baPsd) { this.baPsd = baPsd; }

    public double getDtPsd() { return dtPsd; }
    public void setDtPsd(double dtPsd) { this.dtPsd = dtPsd; }

    public double getSgPsd() { return sgPsd; }
    public void setSgPsd(double sgPsd) { this.sgPsd = sgPsd; }

    public double getSaPsd() { return saPsd; }
    public void setSaPsd(double saPsd) { this.saPsd = saPsd; }

    public double getRgPsd() { return rgPsd; }
    public void setRgPsd(double rgPsd) { this.rgPsd = rgPsd; }

    public double getRaPsd() { return raPsd; }
    public void setRaPsd(double raPsd) { this.raPsd = raPsd; }

    public double getOsPsd() { return osPsd; }
    public void setOsPsd(double osPsd) { this.osPsd = osPsd; }

    public double getOlPsd() { return olPsd; }
    public void setOlPsd(double olPsd) { this.olPsd = olPsd; }

    public double getOaPsd() { return oaPsd; }
    public void setOaPsd(double oaPsd) { this.oaPsd = oaPsd; }

    public double getClkPsd() { return clkPsd; }
    public void setClkPsd(double clkPsd) { this.clkPsd = clkPsd; }

    public double getClkrPsd() { return clkrPsd; }
    public void setClkrPsd(double clkrPsd) { this.clkrPsd = clkrPsd; }

    public boolean isEstimateBa() { return estimateBa; }
    public void setEstimateBa(boolean estimateBa) { this.estimateBa = estimateBa; }

    public boolean isEstimateBg() { return estimateBg; }
    public void setEstimateBg(boolean estimateBg) { this.estimateBg = estimateBg; }

    public boolean isEstimateDt() { return estimateDt; }
    public void setEstimateDt(boolean estimateDt) { this.estimateDt = estimateDt; }

    public boolean isEstimateSg() { return estimateSg; }
    public void setEstimateSg(boolean estimateSg) { this.estimateSg = estimateSg; }

    public boolean isEstimateSa() { return estimateSa; }
    public void setEstimateSa(boolean estimateSa) { this.estimateSa = estimateSa; }

    public boolean isEstimateRg() { return estimateRg; }
    public void setEstimateRg(boolean estimateRg) { this.estimateRg = estimateRg; }

    public boolean isEstimateRa() { return estimateRa; }
    public void setEstimateRa(boolean estimateRa) { this.estimateRa = estimateRa; }

    public boolean isEstimateLever() { return estimateLever; }
    public void setEstimateLever(boolean estimateLever) { this.estimateLever = estimateLever; }

    public boolean isEstimateOdo() { return estimateOdo; }
    public void setEstimateOdo(boolean estimateOdo) { this.estimateOdo = estimateOdo; }

    public boolean isEstimateMagnetometer() { return estimateMagnetometer; }
    public void setEstimateMagnetometer(boolean estimateMagnetometer) { this.estimateMagnetometer = estimateMagnetometer; }

    public boolean isEnableNhc() { return enableNhc; }
    public void setEnableNhc(boolean enableNhc) { this.enableNhc = enableNhc; }

    public boolean isEnableZvu() { return enableZvu; }
    public void setEnableZvu(boolean enableZvu) { this.enableZvu = enableZvu; }

    public boolean isEnableZaru() { return enableZaru; }
    public void setEnableZaru(boolean enableZaru) { this.enableZaru = enableZaru; }

    public boolean isEnableOdo() { return enableOdo; }
    public void setEnableOdo(boolean enableOdo) { this.enableOdo = enableOdo; }

    public boolean isEnableMagnetometer() { return enableMagnetometer; }
    public void setEnableMagnetometer(boolean enableMagnetometer) { this.enableMagnetometer = enableMagnetometer; }

    public double[] getLeverArm() { return leverArm; }
    public void setLeverArm(double[] leverArm) { this.leverArm = leverArm; }

    public double getOdoScale() { return odoScale; }
    public void setOdoScale(double odoScale) { this.odoScale = odoScale; }

    public double[] getOdoLever() { return odoLever; }
    public void setOdoLever(double[] odoLever) { this.odoLever = odoLever; }

    public double getOdoAngle() { return odoAngle; }
    public void setOdoAngle(double odoAngle) { this.odoAngle = odoAngle; }
}