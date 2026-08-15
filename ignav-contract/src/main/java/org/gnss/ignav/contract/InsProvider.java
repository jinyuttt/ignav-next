package org.gnss.ignav.contract;

public interface InsProvider {

    void timeUpdate(ImuMeasurement imu);

    InsPrediction getPrediction();

    void applyCorrection(StateCorrection correction);

    InsSolution getSolution();

    void configure(InsConfig config);

    void setTimeProvider(TimeProvider timeProvider);

    SystemHealth getHealth();

    int getSupportedContractVersion();
}