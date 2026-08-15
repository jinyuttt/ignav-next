package org.gnss.ignav.contract;

public interface TimeProvider {

    GTime getCurrentTime();

    GTime imuToSystem(GTime imuTime);

    GTime gnssToSystem(GTime gnssTime);

    void setTimeBias(double biasSec);

    double getTimeBias();

    double getImuTimeDrift();

    double getGnssTimeDrift();
}