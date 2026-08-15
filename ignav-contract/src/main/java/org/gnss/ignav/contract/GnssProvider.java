package org.gnss.ignav.contract;

public interface GnssProvider {

    void setInsPrediction(InsPrediction prediction);

    GnssObservation computeObservation();

    GnssPositionSolution solvePosition();

    void configure(GnssConfig config);

    void setTimeProvider(TimeProvider timeProvider);

    SystemHealth getHealth();

    int getAvailableSatellites();

    double getGDOP();

    int getSupportedContractVersion();
}