package org.gnss.ignav.fusion;

import org.gnss.ignav.contract.FusionMode;
import org.gnss.ignav.contract.GTime;
import org.gnss.ignav.contract.GnssConfig;
import org.gnss.ignav.contract.GnssObservation;
import org.gnss.ignav.contract.GnssPositionSolution;
import org.gnss.ignav.contract.GnssProvider;
import org.gnss.ignav.contract.ImuMeasurement;
import org.gnss.ignav.contract.InsConfig;
import org.gnss.ignav.contract.InsPrediction;
import org.gnss.ignav.contract.InsProvider;
import org.gnss.ignav.contract.InsSolution;
import org.gnss.ignav.contract.StateCorrection;
import org.gnss.ignav.contract.SystemHealth;
import org.gnss.ignav.contract.TimeProvider;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class IgnavFusion {

    private static final Logger logger = LoggerFactory.getLogger(IgnavFusion.class);

    private final GnssProvider gnssProvider;
    private final InsProvider insProvider;
    private final EkfFusion ekf;
    private final FusionConfig config;
    private final FusionTimeProvider timeProvider;

    private FusionMode.Mode currentMode;
    private InsSolution lastInsSolution;
    private GnssPositionSolution lastGnssSolution;
    private InsSolution lastFusedSolution;
    private SystemHealth systemHealth;
    private long epochCount;
    private boolean initialized;
    private GTime lastImuTime;
    private GTime lastGnssTime;
    private GTime lastFusionTime;

    public IgnavFusion(GnssProvider gnssProvider, InsProvider insProvider, FusionConfig config) {
        this.gnssProvider = gnssProvider;
        this.insProvider = insProvider;
        this.config = config != null ? config : new FusionConfig();
        this.ekf = new EkfFusion();
        this.timeProvider = new FusionTimeProvider();
        this.currentMode = FusionMode.Mode.LC;
        this.lastInsSolution = new InsSolution();
        this.lastGnssSolution = new GnssPositionSolution();
        this.lastFusedSolution = new InsSolution();
        this.systemHealth = new SystemHealth();
        this.epochCount = 0;
        this.initialized = false;
        this.lastImuTime = new GTime();
        this.lastGnssTime = new GTime();
        this.lastFusionTime = new GTime();

        this.gnssProvider.setTimeProvider(timeProvider);
        this.insProvider.setTimeProvider(timeProvider);
    }

    public void init(InsConfig insConfig, GnssConfig gnssConfig) {
        if (insConfig == null || gnssConfig == null) {
            logger.error("Cannot initialize: InsConfig or GnssConfig is null");
            return;
        }

        insProvider.configure(insConfig);
        gnssProvider.configure(gnssConfig);

        config.setInsConfig(insConfig);
        config.setGnssConfig(gnssConfig);

        ekf.init(config.getStateDimension(), null, null);

        initialized = true;
        logger.info("IgnavFusion initialized: mode={}, nx={}",
            config.getFusionMode().getMode(), config.getStateDimension());
    }

    public void processImu(ImuMeasurement imu) {
        if (!initialized || imu == null) return;

        timeProvider.feedImuTime(imu.getTime());
        insProvider.timeUpdate(imu);

        InsPrediction prediction = insProvider.getPrediction();
        if (prediction != null) {
            gnssProvider.setInsPrediction(prediction);
        }

        lastImuTime = new GTime(imu.getTime());
        epochCount++;
    }

    public void processGnss() {
        if (!initialized) return;

        GnssPositionSolution gnssSol = gnssProvider.solvePosition();
        if (gnssSol != null && gnssSol.isValid()) {
            lastGnssSolution = gnssSol;
            lastGnssTime = new GTime(gnssSol.getTime());
            timeProvider.feedGnssTime(gnssSol.getTime());
        }

        updateFusionMode();

        switch (currentMode) {
            case TC:
                processTcUpdate();
                break;
            case STC:
                processStcUpdate();
                break;
            case LC:
                processLcUpdate();
                break;
            case INS_ONLY:
            default:
                processInsOnly();
                break;
        }

        lastFusionTime = timeProvider.getCurrentTime();
    }

    private void processTcUpdate() {
        GnssObservation obs = gnssProvider.computeObservation();
        if (obs == null || obs.getNm() <= 0) {
            logger.debug("TC: no valid GNSS observation, INS-only propagation");
            computeFusedFromIns();
            return;
        }

        InsPrediction pred = insProvider.getPrediction();
        if (pred == null) return;

        double dt = 0.0;
        if (lastFusionTime != null) {
            dt = pred.getTime().diff(lastFusionTime);
        }
        if (dt <= 0 || dt > 1.0) dt = 0.02;

        double[] Qdiag = buildQDiagonal(dt);
        ekf.predict(dt, Qdiag);

        StateCorrection correction = ekf.tcUpdate(obs);
        if (correction != null) {
            if (config.isEnableFeedbackCorrection()) {
                insProvider.applyCorrection(correction);
            }
            logger.debug("TC update: nm={}, innovation={}", obs.getNm(),
                ekf.getLastInnovationRatio());
        }

        computeFusedSolution();
    }

    private void processStcUpdate() {
        GnssPositionSolution gnssSol = lastGnssSolution;
        InsPrediction insPred = insProvider.getPrediction();

        if (gnssSol == null || !gnssSol.isValid() || insPred == null) {
            computeFusedFromIns();
            return;
        }

        double dt = 0.0;
        if (lastFusionTime != null) {
            dt = insPred.getTime().diff(lastFusionTime);
        }
        if (dt <= 0 || dt > 1.0) dt = 0.02;

        double[] Qdiag = buildQDiagonal(dt);
        ekf.predict(dt, Qdiag);

        GnssObservation obs = buildStcObservation(gnssSol, insPred);
        if (obs != null && obs.getNm() > 0) {
            StateCorrection correction = ekf.lcUpdate(obs);
            if (correction != null && config.isEnableFeedbackCorrection()) {
                insProvider.applyCorrection(correction);
            }
        }

        computeFusedSolution();
    }

    private void processLcUpdate() {
        GnssPositionSolution gnssSol = lastGnssSolution;
        InsPrediction insPred = insProvider.getPrediction();

        if (gnssSol == null || !gnssSol.isValid() || insPred == null) {
            computeFusedFromIns();
            return;
        }

        double gnssAge = computeGnssAge(gnssSol);
        if (gnssAge > config.getMaxGnssAgeForLc()) {
            logger.debug("LC: GNSS age too large ({:.1f}s), INS-only", gnssAge);
            computeFusedFromIns();
            return;
        }

        double dt = 0.0;
        if (lastFusionTime != null) {
            dt = insPred.getTime().diff(lastFusionTime);
        }
        if (dt <= 0 || dt > 1.0) dt = 0.02;

        double[] Qdiag = buildQDiagonal(dt);
        ekf.predict(dt, Qdiag);

        GnssObservation obs = buildLcObservation(gnssSol, insPred);
        if (obs != null && obs.getNm() > 0) {
            StateCorrection correction = ekf.lcUpdate(obs);
            if (correction != null && config.isEnableFeedbackCorrection()) {
                insProvider.applyCorrection(correction);
            }
        }

        computeFusedSolution();
    }

    private void processInsOnly() {
        computeFusedFromIns();

        double covTrace = ekf.getCovTrace();
        if (covTrace > config.getMaxInsCovForReset()) {
            logger.warn("INS covariance too large ({:.2f}), considering reset", covTrace);
        }
    }

    private GnssObservation buildLcObservation(GnssPositionSolution gnssSol, InsPrediction insPred) {
        if (gnssSol.getPosEcef() == null || insPred.getPosEcef() == null) return null;

        int nx = config.getStateDimension();
        int nm = 3;
        int IP = 6;

        double[] v = new double[nm];
        double[] H = new double[nm * nx];
        double[] R = new double[nm * nm];

        for (int i = 0; i < 3; i++) {
            v[i] = gnssSol.getPosEcef()[i] - insPred.getPosEcef()[i];
        }

        for (int i = 0; i < nm; i++) {
            for (int j = 0; j < nx; j++) {
                H[i * nx + j] = (j == IP + i) ? -1.0 : 0.0;
            }
        }

        double posNoise = config.getGnssConfig().getPosMeasurementNoise();
        for (int i = 0; i < nm; i++) {
            R[i * nm + i] = posNoise * posNoise;
        }

        if (gnssSol.getPosCov() != null) {
            for (int i = 0; i < 3; i++) {
                R[i * nm + i] += gnssSol.getPosCov()[i * 3 + i];
            }
        }

        return new GnssObservation(gnssSol.getTime(), v, H, R, nm, nx);
    }

    private GnssObservation buildStcObservation(GnssPositionSolution gnssSol, InsPrediction insPred) {
        if (gnssSol.getPosEcef() == null || insPred.getPosEcef() == null) return null;

        int nx = config.getStateDimension();
        boolean hasVel = gnssSol.getVelEcef() != null && insPred.getVelEcef() != null;
        int nm = hasVel ? 6 : 3;
        int IP = 6;
        int IV = 3;

        double[] v = new double[nm];
        double[] H = new double[nm * nx];
        double[] R = new double[nm * nm];

        for (int i = 0; i < 3; i++) {
            v[i] = gnssSol.getPosEcef()[i] - insPred.getPosEcef()[i];
        }

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < nx; j++) {
                H[i * nx + j] = (j == IP + i) ? -1.0 : 0.0;
            }
        }

        double posNoise = config.getGnssConfig().getPosMeasurementNoise();
        for (int i = 0; i < 3; i++) {
            R[i * nm + i] = posNoise * posNoise;
        }

        if (gnssSol.getPosCov() != null) {
            for (int i = 0; i < 3; i++) {
                R[i * nm + i] += gnssSol.getPosCov()[i * 3 + i];
            }
        }

        if (hasVel) {
            for (int i = 0; i < 3; i++) {
                v[3 + i] = gnssSol.getVelEcef()[i] - insPred.getVelEcef()[i];
            }

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < nx; j++) {
                    H[(3 + i) * nx + j] = (j == IV + i) ? -1.0 : 0.0;
                }
            }

            double velNoise = config.getGnssConfig().getVelMeasurementNoise();
            for (int i = 0; i < 3; i++) {
                R[(3 + i) * nm + (3 + i)] = velNoise * velNoise;
            }

            if (gnssSol.getVelCov() != null) {
                for (int i = 0; i < 3; i++) {
                    R[(3 + i) * nm + (3 + i)] += gnssSol.getVelCov()[i * 3 + i];
                }
            }
        }

        return new GnssObservation(gnssSol.getTime(), v, H, R, nm, nx);
    }

    private double[] buildQDiagonal(double dt) {
        int nx = config.getStateDimension();
        double[] Qdiag = new double[nx];

        InsConfig insCfg = config.getInsConfig();
        if (insCfg != null) {
            double gyroPsd = insCfg.getGyroNoisePsd();
            double acclPsd = insCfg.getAcclNoisePsd();
            double bgPsd = insCfg.getBgPsd();
            double baPsd = insCfg.getBaPsd();

            for (int i = 0; i < 3 && i < nx; i++) Qdiag[i] = gyroPsd * gyroPsd;
            for (int i = 3; i < 6 && i < nx; i++) Qdiag[i] = acclPsd * acclPsd;
            for (int i = 6; i < 9 && i < nx; i++) Qdiag[i] = 0.0;
            for (int i = 9; i < 12 && i < nx; i++) Qdiag[i] = baPsd * baPsd;
            for (int i = 12; i < 15 && i < nx; i++) Qdiag[i] = bgPsd * bgPsd;
            for (int i = 15; i < nx; i++) Qdiag[i] = 1e-10;
        } else {
            for (int i = 0; i < 3 && i < nx; i++) Qdiag[i] = 1e-2;
            for (int i = 3; i < 6 && i < nx; i++) Qdiag[i] = 1e-4;
            for (int i = 6; i < 9 && i < nx; i++) Qdiag[i] = 0.0;
            for (int i = 9; i < 12 && i < nx; i++) Qdiag[i] = 1e-6;
            for (int i = 12; i < 15 && i < nx; i++) Qdiag[i] = 1e-6;
            for (int i = 15; i < nx; i++) Qdiag[i] = 1e-10;
        }

        return Qdiag;
    }

    private void computeFusedSolution() {
        InsPrediction pred = insProvider.getPrediction();
        if (pred == null) return;

        lastFusedSolution.setTime(pred.getTime());
        lastFusedSolution.setPosEcef(pred.getPosEcef().clone());
        lastFusedSolution.setVelEcef(pred.getVelEcef().clone());
        lastFusedSolution.setAttQuat(pred.getAttQuat().clone());
        lastFusedSolution.setStatus(InsSolution.STATUS_NAVIGATING);

        double[] ekfState = ekf.getState();
        if (ekfState != null && ekfState.length >= 9) {
            double[] pos = lastFusedSolution.getPosEcef();
            double[] vel = lastFusedSolution.getVelEcef();
            for (int i = 0; i < 3; i++) {
                pos[i] -= ekfState[6 + i];
                vel[i] -= ekfState[3 + i];
            }
        }

        double[] ekfCov = ekf.getCovariance();
        int nx = ekf.getStateDimension();
        if (ekfCov != null && nx >= 9) {
            double[] posCov = lastFusedSolution.getPosCov();
            double[] velCov = lastFusedSolution.getVelCov();
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    posCov[i * 3 + j] = pred.getPosCov()[i * 3 + j] + ekfCov[(6 + i) * nx + (6 + j)];
                    velCov[i * 3 + j] = pred.getVelCov()[i * 3 + j] + ekfCov[(3 + i) * nx + (3 + j)];
                }
            }
        }
    }

    private void computeFusedFromIns() {
        InsSolution insSol = insProvider.getSolution();
        if (insSol == null) return;

        lastInsSolution = insSol;
        lastFusedSolution.setTime(insSol.getTime());
        lastFusedSolution.setPosEcef(insSol.getPosEcef().clone());
        lastFusedSolution.setVelEcef(insSol.getVelEcef().clone());
        lastFusedSolution.setAttQuat(insSol.getAttQuat().clone());
        lastFusedSolution.setPosCov(insSol.getPosCov().clone());
        lastFusedSolution.setVelCov(insSol.getVelCov().clone());
        lastFusedSolution.setAttCov(insSol.getAttCov().clone());
        lastFusedSolution.setStatus(insSol.getStatus());
    }

    private void updateFusionMode() {
        if (!config.isEnableAdaptiveMode()) {
            currentMode = config.getFusionMode().getMode();
            return;
        }

        int numSat = gnssProvider.getAvailableSatellites();
        double gdop = gnssProvider.getGDOP();

        FusionMode modeConfig = config.getFusionMode();
        currentMode = modeConfig.determineMode(numSat, gdop);

        if (epochCount % 100 == 0) {
            logger.debug("Fusion mode: {}, sats={}, gdop={:.2f}", currentMode, numSat, gdop);
        }
    }

    private double computeGnssAge(GnssPositionSolution sol) {
        if (sol == null || sol.getTime() == null) return Double.MAX_VALUE;
        GTime now = timeProvider.getCurrentTime();
        return Math.abs(now.diff(sol.getTime()));
    }

    private void updateSystemHealth() {
        SystemHealth insHealth = insProvider.getHealth();
        SystemHealth gnssHealth = gnssProvider.getHealth();

        systemHealth.setGnssAvailableSats(gnssHealth.getGnssAvailableSats());
        systemHealth.setGnssAge(gnssHealth.getGnssAge());
        systemHealth.setInsCovTrace(insHealth.getInsCovTrace());
        systemHealth.setGnssCovTrace(gnssHealth.getGnssCovTrace());
        systemHealth.setInnovationRatio(ekf.getLastInnovationRatio());
        systemHealth.setTimestampMs(System.currentTimeMillis());

        boolean insOk = insHealth.getStatus() == SystemHealth.HealthStatus.NOMINAL
            || insHealth.getStatus() == SystemHealth.HealthStatus.INS_DEGRADED;
        boolean gnssOk = gnssHealth.getStatus() == SystemHealth.HealthStatus.NOMINAL
            || gnssHealth.getStatus() == SystemHealth.HealthStatus.GNSS_DEGRADED;

        if (insOk && gnssOk) {
            systemHealth.setStatus(SystemHealth.HealthStatus.NOMINAL);
            systemHealth.setReason("INS+GNSS nominal");
        } else if (insOk && !gnssOk) {
            systemHealth.setStatus(SystemHealth.HealthStatus.INS_ONLY);
            systemHealth.setReason("GNSS unavailable: " + gnssHealth.getReason());
        } else if (!insOk && gnssOk) {
            systemHealth.setStatus(SystemHealth.HealthStatus.GNSS_ONLY);
            systemHealth.setReason("INS degraded: " + insHealth.getReason());
        } else {
            systemHealth.setStatus(SystemHealth.HealthStatus.FAILED);
            systemHealth.setReason("Both INS and GNSS failed");
        }
    }

    public InsSolution getFusedSolution() {
        return lastFusedSolution;
    }

    public InsSolution getInsSolution() {
        return lastInsSolution;
    }

    public GnssPositionSolution getGnssSolution() {
        return lastGnssSolution;
    }

    public FusionMode.Mode getCurrentMode() {
        return currentMode;
    }

    public SystemHealth getSystemHealth() {
        updateSystemHealth();
        return systemHealth;
    }

    public TimeProvider getTimeProvider() {
        return timeProvider;
    }

    public EkfFusion getEkf() {
        return ekf;
    }

    public FusionConfig getConfig() {
        return config;
    }

    public long getEpochCount() {
        return epochCount;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void reset() {
        ekf.init(config.getStateDimension(), null, null);
        gnssProvider.setInsPrediction(null);
        epochCount = 0;
        lastImuTime = new GTime();
        lastGnssTime = new GTime();
        lastFusionTime = new GTime();
        lastFusedSolution = new InsSolution();
        logger.info("IgnavFusion reset");
    }
}