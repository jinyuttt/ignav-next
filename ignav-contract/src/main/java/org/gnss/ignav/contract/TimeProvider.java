package org.gnss.ignav.contract;

public interface TimeProvider {

    GTime getCurrentTime();

    GTime imuToSystem(GTime imuTime);

    GTime gnssToSystem(GTime gnssTime);

    void setTimeBias(double biasSec);

    double getTimeBias();

    double getImuTimeDrift();

    double getGnssTimeDrift();

    double getTimeDriftRate();

    void setTimeDriftRate(double rate);

    boolean isSecondOrderModel();

    void setSecondOrderModel(boolean enabled);
}