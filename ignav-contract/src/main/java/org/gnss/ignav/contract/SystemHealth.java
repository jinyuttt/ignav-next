package org.gnss.ignav.contract;

import java.io.Serializable;

public class SystemHealth implements Serializable {

    private static final long serialVersionUID = 1L;

    public enum HealthStatus {
        NOMINAL,
        INS_ONLY,
        GNSS_ONLY,
        INS_DEGRADED,
        GNSS_DEGRADED,
        DEGRADED,
        FAILED
    }

    private HealthStatus status;
    private String reason;
    private double insCovTrace;
    private double gnssCovTrace;
    private double innovationRatio;
    private int gnssAvailableSats;
    private double gnssAge;
    private long timestampMs;

    public SystemHealth() {
        this.status = HealthStatus.NOMINAL;
        this.reason = "";
        this.insCovTrace = 0.0;
        this.gnssCovTrace = 0.0;
        this.innovationRatio = 0.0;
        this.gnssAvailableSats = 0;
        this.gnssAge = 0.0;
        this.timestampMs = System.currentTimeMillis();
    }

    public HealthStatus getStatus() { return status; }
    public void setStatus(HealthStatus status) { this.status = status; }

    public String getReason() { return reason; }
    public void setReason(String reason) { this.reason = reason; }

    public double getInsCovTrace() { return insCovTrace; }
    public void setInsCovTrace(double insCovTrace) { this.insCovTrace = insCovTrace; }

    public double getGnssCovTrace() { return gnssCovTrace; }
    public void setGnssCovTrace(double gnssCovTrace) { this.gnssCovTrace = gnssCovTrace; }

    public double getInnovationRatio() { return innovationRatio; }
    public void setInnovationRatio(double innovationRatio) { this.innovationRatio = innovationRatio; }

    public int getGnssAvailableSats() { return gnssAvailableSats; }
    public void setGnssAvailableSats(int gnssAvailableSats) { this.gnssAvailableSats = gnssAvailableSats; }

    public double getGnssAge() { return gnssAge; }
    public void setGnssAge(double gnssAge) { this.gnssAge = gnssAge; }

    public long getTimestampMs() { return timestampMs; }
    public void setTimestampMs(long timestampMs) { this.timestampMs = timestampMs; }

    public boolean isNavigable() {
        return status == HealthStatus.NOMINAL
                || status == HealthStatus.INS_ONLY
                || status == HealthStatus.GNSS_ONLY
                || status == HealthStatus.INS_DEGRADED
                || status == HealthStatus.GNSS_DEGRADED
                || status == HealthStatus.DEGRADED;
    }

    @Override
    public String toString() {
        return "SystemHealth{status=" + status + ", reason='" + reason + "'" +
               ", insCov=" + String.format("%.4f", insCovTrace) +
               ", gnssSats=" + gnssAvailableSats +
               ", gnssAge=" + String.format("%.1f", gnssAge) + "s}";
    }
}