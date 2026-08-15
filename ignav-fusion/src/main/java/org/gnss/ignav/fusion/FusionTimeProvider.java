package org.gnss.ignav.fusion;

import org.gnss.ignav.contract.GTime;
import org.gnss.ignav.contract.TimeProvider;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class FusionTimeProvider implements TimeProvider {

    private static final Logger logger = LoggerFactory.getLogger(FusionTimeProvider.class);

    private GTime systemTime;
    private double imuTimeBias;
    private double gnssTimeBias;
    private double imuTimeDrift;
    private double gnssTimeDrift;
    private GTime lastImuTime;
    private GTime lastGnssTime;
    private int imuCount;
    private int gnssCount;
    private boolean gnssTimeValid;

    public FusionTimeProvider() {
        this.systemTime = new GTime();
        this.imuTimeBias = 0.0;
        this.gnssTimeBias = 0.0;
        this.imuTimeDrift = 0.0;
        this.gnssTimeDrift = 0.0;
        this.lastImuTime = new GTime();
        this.lastGnssTime = new GTime();
        this.imuCount = 0;
        this.gnssCount = 0;
        this.gnssTimeValid = false;
    }

    public void feedImuTime(GTime imuTime) {
        if (imuTime == null) return;

        if (imuCount == 0) {
            systemTime = new GTime(imuTime);
            lastImuTime = new GTime(imuTime);
        } else {
            double dt = imuTime.diff(lastImuTime);
            if (dt > 0 && dt < 1.0) {
                imuTimeDrift = imuTimeDrift * 0.99 + dt * 0.01;
            }
            lastImuTime = new GTime(imuTime);
        }

        if (!gnssTimeValid) {
            systemTime = new GTime(imuTime);
        }

        imuCount++;
    }

    public void feedGnssTime(GTime gnssTime) {
        if (gnssTime == null) return;

        if (gnssCount == 0) {
            lastGnssTime = new GTime(gnssTime);
        } else {
            double dt = gnssTime.diff(lastGnssTime);
            if (dt > 0 && dt < 3600.0) {
                gnssTimeDrift = gnssTimeDrift * 0.99 + dt * 0.01;
            }
            lastGnssTime = new GTime(gnssTime);
        }

        if (gnssCount < 10 || !gnssTimeValid) {
            systemTime = new GTime(gnssTime);
            gnssTimeValid = true;
        } else {
            double bias = gnssTime.diff(systemTime);
            imuTimeBias = imuTimeBias * 0.95 + bias * 0.05;
        }

        gnssCount++;
    }

    @Override
    public GTime getCurrentTime() {
        return new GTime(systemTime);
    }

    @Override
    public GTime imuToSystem(GTime imuTime) {
        if (imuTime == null) return new GTime();
        GTime result = new GTime(imuTime);
        result.sec += imuTimeBias;
        normalize(result);
        return result;
    }

    @Override
    public GTime gnssToSystem(GTime gnssTime) {
        if (gnssTime == null) return new GTime();
        GTime result = new GTime(gnssTime);
        result.sec += gnssTimeBias;
        normalize(result);
        return result;
    }

    @Override
    public void setTimeBias(double biasSec) {
        this.gnssTimeBias = biasSec;
        logger.debug("Time bias updated: {}s", biasSec);
    }

    @Override
    public double getTimeBias() {
        return gnssTimeBias;
    }

    @Override
    public double getImuTimeDrift() {
        return imuTimeDrift;
    }

    @Override
    public double getGnssTimeDrift() {
        return gnssTimeDrift;
    }

    public boolean isGnssTimeValid() {
        return gnssTimeValid;
    }

    private void normalize(GTime t) {
        while (t.sec >= 86400.0) {
            t.sec -= 86400.0;
            t.time++;
        }
        while (t.sec < 0.0) {
            t.sec += 86400.0;
            t.time--;
        }
    }
}