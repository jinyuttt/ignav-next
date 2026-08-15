package org.gnss.ignav.app;

import org.gnss.ignav.contract.FusionMode;
import org.gnss.ignav.contract.GTime;
import org.gnss.ignav.contract.GnssConfig;
import org.gnss.ignav.contract.GnssPositionSolution;
import org.gnss.ignav.contract.ImuMeasurement;
import org.gnss.ignav.contract.InsConfig;
import org.gnss.ignav.contract.InsSolution;
import org.gnss.ignav.contract.SystemHealth;
import org.gnss.ignav.fusion.FusionConfig;
import org.gnss.ignav.fusion.IgnavFusion;
import org.gnss.ignav.gnss.GnssProviderImpl;
import org.gnss.ignav.ins.InsProviderImpl;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;

public class IgnavApp {

    private static final Logger logger = LoggerFactory.getLogger(IgnavApp.class);

    private IgnavFusion fusion;
    private boolean running;
    private long totalImuCount;
    private long totalGnssCount;
    private long startTimeMs;
    private SolutionLogger solutionLogger;

    public IgnavApp() {
        this.running = false;
        this.totalImuCount = 0;
        this.totalGnssCount = 0;
        this.startTimeMs = 0;
        this.solutionLogger = new SolutionLogger();
    }

    public void init(InsConfig insConfig, GnssConfig gnssConfig, FusionConfig fusionConfig) {
        GnssProviderImpl gnssProvider = new GnssProviderImpl();
        InsProviderImpl insProvider = new InsProviderImpl();

        this.fusion = new IgnavFusion(gnssProvider, insProvider, fusionConfig);
        this.fusion.init(insConfig, gnssConfig);

        logger.info("IgnavApp initialized successfully");
    }

    public void feedImu(ImuMeasurement imu) {
        if (fusion == null || !fusion.isInitialized()) {
            logger.warn("Fusion not initialized, skipping IMU data");
            return;
        }

        fusion.processImu(imu);
        totalImuCount++;

        if (totalImuCount % 500 == 0) {
            InsSolution sol = fusion.getFusedSolution();
            logger.debug("IMU #{}, pos=[{},{},{}], mode={}",
                totalImuCount,
                sol.getPosEcef()[0], sol.getPosEcef()[1], sol.getPosEcef()[2],
                fusion.getCurrentMode());
        }
    }

    public void feedGnss() {
        if (fusion == null || !fusion.isInitialized()) {
            logger.warn("Fusion not initialized, skipping GNSS update");
            return;
        }

        fusion.processGnss();
        totalGnssCount++;

        InsSolution fused = fusion.getFusedSolution();
        GnssPositionSolution gnss = fusion.getGnssSolution();
        SystemHealth health = fusion.getSystemHealth();

        solutionLogger.log(fused, gnss, health, fusion.getCurrentMode());

        if (totalGnssCount % 10 == 0) {
            logger.info("GNSS #{}, mode={}, sats={}, health={}, pos=[{},{},{}]",
                totalGnssCount, fusion.getCurrentMode(),
                health.getGnssAvailableSats(), health.getStatus(),
                fused.getPosEcef()[0], fused.getPosEcef()[1], fused.getPosEcef()[2]);
        }
    }

    public void runOffline(String imuFilePath, String gnssObsPath, String gnssNavPath) {
        if (fusion == null) {
            logger.error("Fusion not initialized, call init() first");
            return;
        }

        running = true;
        startTimeMs = System.currentTimeMillis();

        logger.info("Starting offline processing...");
        logger.info("  IMU file: {}", imuFilePath);
        logger.info("  GNSS obs: {}", gnssObsPath);
        logger.info("  GNSS nav: {}", gnssNavPath);

        List<ImuMeasurement> imuList = loadImuFile(imuFilePath);
        if (imuList.isEmpty()) {
            logger.error("No IMU data loaded");
            return;
        }

        logger.info("Loaded {} IMU measurements", imuList.size());

        boolean gnssLoaded = false;
        if (gnssObsPath != null && !gnssObsPath.isEmpty()) {
            gnssLoaded = loadGnssObs(gnssObsPath);
        }
        if (gnssNavPath != null && !gnssNavPath.isEmpty()) {
            loadGnssNav(gnssNavPath);
        }

        int imuIdx = 0;
        int gnssInterval = 0;

        while (running && imuIdx < imuList.size()) {
            ImuMeasurement imu = imuList.get(imuIdx);
            feedImu(imu);
            imuIdx++;

            gnssInterval++;
            if (gnssInterval >= 5 && gnssLoaded) {
                feedGnss();
                gnssInterval = 0;
            }
        }

        if (gnssLoaded) {
            feedGnss();
        }

        long elapsed = System.currentTimeMillis() - startTimeMs;
        logger.info("Offline processing complete: {} IMU, {} GNSS updates in {}ms",
            totalImuCount, totalGnssCount, elapsed);

        running = false;
    }

    public void stop() {
        running = false;
        logger.info("IgnavApp stopped");
    }

    public InsSolution getFusedSolution() {
        return fusion != null ? fusion.getFusedSolution() : new InsSolution();
    }

    public SystemHealth getSystemHealth() {
        return fusion != null ? fusion.getSystemHealth() : new SystemHealth();
    }

    public FusionMode.Mode getCurrentMode() {
        return fusion != null ? fusion.getCurrentMode() : FusionMode.Mode.INS_ONLY;
    }

    public boolean isRunning() {
        return running;
    }

    public IgnavFusion getFusion() {
        return fusion;
    }

    private boolean loadGnssObs(String path) {
        return false;
    }

    private boolean loadGnssNav(String path) {
        return false;
    }

    private List<ImuMeasurement> loadImuFile(String path) {
        List<ImuMeasurement> list = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(path))) {
            String line;
            while ((line = reader.readLine()) != null) {
                if (line.startsWith("%") || line.startsWith("#") || line.trim().isEmpty()) {
                    continue;
                }

                StringTokenizer st = new StringTokenizer(line);
                if (st.countTokens() < 8) continue;

                long week = Long.parseLong(st.nextToken());
                double tow = Double.parseDouble(st.nextToken());

                double[] gyro = new double[3];
                double[] accl = new double[3];
                for (int i = 0; i < 3; i++) gyro[i] = Double.parseDouble(st.nextToken());
                for (int i = 0; i < 3; i++) accl[i] = Double.parseDouble(st.nextToken());

                GTime time = new GTime(week, tow);
                list.add(new ImuMeasurement(time, gyro, accl));
            }
            logger.info("Parsed {} IMU records from {}", list.size(), path);
        } catch (Exception e) {
            logger.error("Failed to load IMU file: {}", e.getMessage());
        }
        return list;
    }

    public static InsConfig createDefaultInsConfig() {
        InsConfig cfg = new InsConfig();

        cfg.setGyroNoisePsd(0.01);
        cfg.setAcclNoisePsd(0.01);
        cfg.setBgPsd(1e-5);
        cfg.setBaPsd(1e-4);
        cfg.setDtPsd(1e-6);
        cfg.setSgPsd(1e-6);
        cfg.setSaPsd(1e-5);
        cfg.setRgPsd(1e-6);
        cfg.setRaPsd(1e-5);

        cfg.setEstimateBa(true);
        cfg.setEstimateBg(true);
        cfg.setEstimateDt(false);
        cfg.setEstimateSg(false);
        cfg.setEstimateSa(false);
        cfg.setEstimateRg(false);
        cfg.setEstimateRa(false);
        cfg.setEstimateLever(false);

        cfg.setEnableNhc(false);
        cfg.setEnableZvu(true);
        cfg.setEnableZaru(false);
        cfg.setEnableOdo(false);

        cfg.setInitAttStd(Math.toRadians(1.0));
        cfg.setInitPosStd(new double[]{5.0, 5.0, 5.0});
        cfg.setInitVelStd(new double[]{0.5, 0.5, 0.5});

        return cfg;
    }

    public static GnssConfig createDefaultGnssConfig() {
        GnssConfig cfg = new GnssConfig();
        cfg.setFusionMode(GnssConfig.FusionMode.LC);
        cfg.setPosMeasurementNoise(2.5);
        cfg.setVelMeasurementNoise(0.1);
        cfg.setMaxPositionInnovation(1000.0);
        cfg.setMaxVelocityInnovation(100.0);
        cfg.setMinSatellitesForTc(5);
        return cfg;
    }

    public static FusionConfig createDefaultFusionConfig() {
        FusionConfig cfg = new FusionConfig();
        cfg.setEnableAdaptiveMode(true);
        cfg.setEnableFeedbackCorrection(true);
        cfg.setEnableSmoothing(false);
        cfg.setStateDimension(15);
        cfg.setChi2Threshold(0.01);
        return cfg;
    }

    public static void main(String[] args) {
        logger.info("========================================");
        logger.info("  ignav-next: INS/GNSS Integrated Navigation");
        logger.info("========================================");

        InsConfig insConfig = createDefaultInsConfig();
        GnssConfig gnssConfig = createDefaultGnssConfig();
        FusionConfig fusionConfig = createDefaultFusionConfig();

        IgnavApp app = new IgnavApp();
        app.init(insConfig, gnssConfig, fusionConfig);

        if (args.length >= 3) {
            String imuFile = args[0];
            String gnssObsFile = args[1];
            String gnssNavFile = args[2];
            app.runOffline(imuFile, gnssObsFile, gnssNavFile);
        } else {
            logger.info("Usage: IgnavApp <imu_file> <gnss_obs_file> <gnss_nav_file>");
            logger.info("Running demo mode with simulated data...");

            runDemo(app);
        }

        InsSolution finalSol = app.getFusedSolution();
        SystemHealth health = app.getSystemHealth();
        logger.info("Final solution: pos=[{},{},{}]",
            finalSol.getPosEcef()[0], finalSol.getPosEcef()[1], finalSol.getPosEcef()[2]);
        logger.info("System health: {}", health);
    }

    private static void runDemo(IgnavApp app) {
        InsConfig insConfig = createDefaultInsConfig();

        double[] initPos = new double[]{-2674691.0, 3745950.0, 4499760.0};
        double[] initVel = new double[]{0.0, 0.0, 0.0};
        double[] initAtt = new double[]{1.0, 0.0, 0.0, 0.0};

        insConfig.setInitPosEcef(initPos);
        insConfig.setInitVelEcef(initVel);
        insConfig.setInitAttQuat(initAtt);

        GnssConfig gnssConfig = createDefaultGnssConfig();
        FusionConfig fusionConfig = createDefaultFusionConfig();

        app.init(insConfig, gnssConfig, fusionConfig);

        long baseWeek = 2300;
        double baseTow = 432000.0;
        double dt = 0.01;
        int numEpochs = 1000;

        for (int i = 0; i < numEpochs; i++) {
            double tow = baseTow + i * dt;
            GTime time = new GTime(baseWeek, tow);

            double[] gyro = new double[]{0.0001 * Math.sin(i * 0.01), 0.00005, 0.00003};
            double[] accl = new double[]{0.01, 0.02, 9.81};
            ImuMeasurement imu = new ImuMeasurement(time, gyro, accl);

            app.feedImu(imu);

            if (i % 5 == 0) {
                app.feedGnss();
            }
        }

        logger.info("Demo complete: {} epochs processed", numEpochs);
    }

    private static class SolutionLogger {
        private long logCount;

        SolutionLogger() {
            this.logCount = 0;
        }

        void log(InsSolution fused, GnssPositionSolution gnss,
                 SystemHealth health, FusionMode.Mode mode) {
            logCount++;

            if (logCount % 10 != 0) return;

            StringBuilder sb = new StringBuilder();
            sb.append(String.format("SOL: t=%d/%.3f", fused.getTime().time, fused.getTime().sec));
            sb.append(String.format(" pos=[%.4f,%.4f,%.4f]",
                fused.getPosEcef()[0], fused.getPosEcef()[1], fused.getPosEcef()[2]));
            sb.append(String.format(" vel=[%.6f,%.6f,%.6f]",
                fused.getVelEcef()[0], fused.getVelEcef()[1], fused.getVelEcef()[2]));
            sb.append(String.format(" mode=%s", mode));
            sb.append(String.format(" sats=%d", health.getGnssAvailableSats()));
            sb.append(String.format(" health=%s", health.getStatus()));

            logger.info(sb.toString());
        }
    }
}