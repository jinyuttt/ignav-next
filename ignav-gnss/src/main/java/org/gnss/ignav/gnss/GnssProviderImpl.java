package org.gnss.ignav.gnss;

import org.gnss.ignav.contract.ContractVersion;
import org.gnss.ignav.contract.GTime;
import org.gnss.ignav.contract.GnssConfig;
import org.gnss.ignav.contract.GnssObservation;
import org.gnss.ignav.contract.GnssPositionSolution;
import org.gnss.ignav.contract.InsPrediction;
import org.gnss.ignav.contract.SystemHealth;
import org.gnss.ignav.contract.TimeProvider;
import org.gnss.ignav.contract.GnssProvider;
import org.rtklib.java.constants.Constants;
import org.rtklib.java.coord.CoordTransform;
import org.rtklib.java.data.Eph;
import org.rtklib.java.data.Geph;
import org.rtklib.java.data.Nav;
import org.rtklib.java.data.Obs;
import org.rtklib.java.data.Obsd;
import org.rtklib.java.data.PrcOpt;
import org.rtklib.java.data.Rtk;
import org.rtklib.java.data.Sol;
import org.rtklib.java.data.Ssr;
import org.rtklib.java.data.Sta;
import org.rtklib.java.ephemeris.EphModel;
import org.rtklib.java.pntpos.PntPos;
import org.rtklib.java.rtkpos.RtkCore;
import org.rtklib.java.rinex.RinexParser;
import org.rtklib.java.rtcm.ObservationEpoch;
import org.rtklib.java.rtcm.Rtcm;
import org.rtklib.java.rtcm.RtcmCallbackDecoder;
import org.rtklib.java.rtcm.RtcmDataHandler;
import org.rtklib.java.rtcm.AuxData;

import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class GnssProviderImpl implements GnssProvider {

    private static final Logger logger = LoggerFactory.getLogger(GnssProviderImpl.class);

    private GnssConfig config;
    private PrcOpt prcOpt;
    private Nav nav;
    private Rtk rtk;
    private Sol lastSol;
    private InsPrediction insPrediction;
    private TimeProvider timeProvider;
    private boolean configured;
    private long lastGnssTimeMs;
    private int lastNumSats;
    private double lastGdop;
    private Obs pendingObs;
    private Rtcm rtcm;
    private RtcmCallbackDecoder rtcmDecoder;
    private static final double MAX_GNSS_AGE = 30.0;

    public GnssProviderImpl() {
        this.config = new GnssConfig();
        this.prcOpt = createDefaultPrcOpt();
        this.nav = new Nav();
        this.rtk = new Rtk();
        this.rtk.opt = prcOpt;
        this.lastSol = new Sol();
        this.insPrediction = null;
        this.timeProvider = null;
        this.configured = false;
        this.lastGnssTimeMs = 0;
        this.lastNumSats = 0;
        this.lastGdop = 0.0;
        this.pendingObs = new Obs();
        this.rtcm = new Rtcm();
        this.rtcmDecoder = new RtcmCallbackDecoder(new RtcmDataHandler() {
            @Override
            public void onStation(Sta sta) {}
            @Override
            public void onSsr(Ssr ssr) {}
            @Override
            public void onEph(Eph eph) {
                addEphToNav(eph);
            }
            @Override
            public void onGeph(Geph geph) {
                addGephToNav(geph);
            }
            @Override
            public void onObservationEpoch(ObservationEpoch epoch) {
                logger.debug("RTCM obs epoch received");
            }
            @Override
            public void onAuxData(AuxData aux) {}
            @Override
            public void onFinish() {}
        });
    }

    @Override
    public void setInsPrediction(InsPrediction prediction) {
        this.insPrediction = prediction;
        if (prediction != null && lastSol != null && lastSol.stat != 0) {
            org.rtklib.java.data.GTime rtkTime = toRtkGTime(prediction.getTime());
            double dt = org.rtklib.java.time.TimeSystem.timediff(rtkTime, lastSol.time);
            if (Math.abs(dt) > config.getMaxSyncTimeDiff()) {
                logger.debug("INS prediction time diff too large: {}s", dt);
            }
        }
    }

    @Override
    public GnssObservation computeObservation() {
        if (rtk == null || rtk.x == null || rtk.nx <= 0) {
            return new GnssObservation();
        }

        int nx = rtk.nx;

        if (insPrediction == null) {
            return new GnssObservation();
        }

        double[] insPos = insPrediction.getPosEcef();
        double[] insVel = insPrediction.getVelEcef();
        if (insPos == null || CoordTransform.norm3(insPos) < 1.0) {
            return new GnssObservation();
        }

        Obsd[] obs = getObsData();
        if (obs == null || obs.length < 1) {
            return new GnssObservation();
        }

        int n = obs.length;
        double[] pos = new double[3];
        CoordTransform.ecef2pos(insPos, pos);

        List<Double> vList = new ArrayList<>();
        List<Double> hList = new ArrayList<>();
        List<Double> rList = new ArrayList<>();
        int nm = 0;

        for (int i = 0; i < n; i++) {
            if (obs[i].P[0] == 0.0) continue;

            double[] rs = new double[6];
            double[] dts = new double[2];
            double[] vare = new double[1];
            EphModel.satpos(lastSol.time, nav, obs[i].sat, rs, dts, vare);

            double[] e = new double[3];
            double r = geodist(rs, insPos, e);
            double[] azelArr = new double[2];
            double el = satazel(pos, e, azelArr);
            if (el < prcOpt.elmin) continue;

            double sinel = Math.sin(el);
            if (sinel <= 0.0) continue;

            double pr = obs[i].P[0];
            if (pr != 0.0) {
                double P = pr - Constants.CLIGHT * dts[0];
                double innov = P - r;

                if (Math.abs(innov) > config.getMaxPositionInnovation()) continue;

                vList.add(innov);
                for (int j = 0; j < 6; j++) hList.add(0.0);
                for (int j = 0; j < 3; j++) hList.add(-e[j]);
                for (int j = 9; j < nx; j++) hList.add(0.0);
                rList.add(config.getPosMeasurementNoise() * config.getPosMeasurementNoise() / (sinel * sinel));
                nm++;
            }

            if (insVel != null && obs[i].D[0] != 0.0f && obs[i].L[0] != 0.0) {
                double freq = prcOpt.freq[0];
                double rate = rs[3] * e[0] + rs[4] * e[1] + rs[5] * e[2];
                double velProj = insVel[0] * e[0] + insVel[1] * e[1] + insVel[2] * e[2];
                double doppler = -freq * (rate - velProj) / Constants.CLIGHT;

                vList.add(doppler);
                for (int j = 0; j < 3; j++) hList.add(0.0);
                for (int j = 0; j < 3; j++) hList.add(-e[j]);
                for (int j = 0; j < 3; j++) hList.add(0.0);
                for (int j = 9; j < nx; j++) hList.add(0.0);
                rList.add(config.getVelMeasurementNoise() * config.getVelMeasurementNoise());
                nm++;
            }
        }

        if (nm <= 0) return new GnssObservation();

        double[] v = new double[nm];
        double[] H = new double[nm * nx];
        double[] R = new double[nm * nm];
        for (int i = 0; i < nm; i++) v[i] = vList.get(i);
        for (int i = 0; i < nm * nx; i++) H[i] = hList.get(i);
        for (int i = 0; i < nm; i++) R[i + i * nm] = rList.get(i);

        return new GnssObservation(toContractGTime(lastSol.time), v, H, R, nm, nx);
    }

    @Override
    public GnssPositionSolution solvePosition() {
        GnssPositionSolution sol = new GnssPositionSolution();

        Obsd[] obs = getObsData();
        if (obs == null || obs.length < 4) {
            return sol;
        }

        int n = obs.length;

        if (config.getFusionMode() == GnssConfig.FusionMode.TC) {
            int result = RtkCore.rtkpos(rtk, obs, n, nav);
            if (result != 0 && rtk.sol != null) {
                copyRtkSolToContract(rtk.sol, sol);
                lastSol = new Sol(rtk.sol);
                lastGnssTimeMs = System.currentTimeMillis();
                lastNumSats = rtk.sol.ns;
                lastGdop = rtk.sol.gdop;
            } else {
                int sppResult = PntPos.pntpos(obs, n, nav, prcOpt, lastSol, null, rtk.ssat);
                if (sppResult != 0) {
                    copyRtkSolToContract(lastSol, sol);
                    lastGnssTimeMs = System.currentTimeMillis();
                    lastNumSats = lastSol.ns;
                    lastGdop = lastSol.gdop;
                }
            }
        } else {
            int result = PntPos.pntpos(obs, n, nav, prcOpt, lastSol, null, rtk.ssat);
            if (result != 0) {
                copyRtkSolToContract(lastSol, sol);
                lastGnssTimeMs = System.currentTimeMillis();
                lastNumSats = lastSol.ns;
                lastGdop = lastSol.gdop;
            }
        }

        return sol;
    }

    @Override
    public void configure(GnssConfig config) {
        if (config != null) {
            this.config = config;
        }
        this.prcOpt = createPrcOptFromConfig(this.config);
        this.rtk = new Rtk();
        this.rtk.opt = prcOpt;
        this.configured = true;
        logger.info("GNSS configured: mode={}, posNoise={}, velNoise={}, minSatsTc={}",
            config.getFusionMode(), config.getPosMeasurementNoise(),
            config.getVelMeasurementNoise(), config.getMinSatellitesForTc());
    }

    @Override
    public void setTimeProvider(TimeProvider timeProvider) {
        this.timeProvider = timeProvider;
    }

    @Override
    public SystemHealth getHealth() {
        SystemHealth health = new SystemHealth();

        double gnssAge = (System.currentTimeMillis() - lastGnssTimeMs) / 1000.0;
        health.setGnssAge(gnssAge);
        health.setGnssAvailableSats(lastNumSats);

        if (lastNumSats >= 6 && lastGdop < 3.0 && gnssAge < 5.0) {
            health.setStatus(SystemHealth.HealthStatus.NOMINAL);
        } else if (lastNumSats >= 4 && gnssAge < 10.0) {
            health.setStatus(SystemHealth.HealthStatus.GNSS_DEGRADED);
            health.setReason("Low satellite count or high GDOP");
        } else if (lastNumSats >= 2 && gnssAge < MAX_GNSS_AGE) {
            health.setStatus(SystemHealth.HealthStatus.GNSS_DEGRADED);
            health.setReason("Degraded GNSS availability");
        } else if (gnssAge >= MAX_GNSS_AGE) {
            health.setStatus(SystemHealth.HealthStatus.INS_ONLY);
            health.setReason("GNSS signal lost for " + String.format("%.1f", gnssAge) + "s");
        } else {
            health.setStatus(SystemHealth.HealthStatus.FAILED);
            health.setReason("No GNSS solution");
        }

        if (lastSol != null && lastSol.qr != null) {
            double posCovTrace = lastSol.qr[0] + lastSol.qr[1] + lastSol.qr[2];
            health.setGnssCovTrace(posCovTrace);
        }

        return health;
    }

    @Override
    public int getAvailableSatellites() {
        return lastNumSats;
    }

    @Override
    public double getGDOP() {
        return lastGdop;
    }

    @Override
    public int getSupportedContractVersion() {
        return ContractVersion.VERSION;
    }

    public boolean loadRinexObs(String filePath) {
        try {
            RinexParser parser = new RinexParser();
            boolean ok = parser.parseObs(filePath);
            if (ok && parser.obs != null && parser.obs.n > 0) {
                this.pendingObs = new Obs(parser.obs);
                logger.info("Loaded {} obs from RINEX: {}", parser.obs.n, filePath);
                return true;
            }
        } catch (Exception e) {
            logger.error("Failed to load RINEX obs: {}", e.getMessage());
        }
        return false;
    }

    public boolean loadRinexNav(String filePath) {
        try {
            RinexParser parser = new RinexParser();
            boolean ok = parser.parseNav(filePath);
            if (ok && parser.nav != null) {
                this.nav = parser.nav;
                logger.info("Loaded nav data from RINEX: {}", filePath);
                return true;
            }
        } catch (Exception e) {
            logger.error("Failed to load RINEX nav: {}", e.getMessage());
        }
        return false;
    }

    public void processRtcmData(byte[] data, int offset, int length) {
        try {
            rtcmDecoder.feed(data, offset, length);
        } catch (Exception e) {
            logger.error("RTCM decode error: {}", e.getMessage());
        }
    }

    public void setObservation(Obs obs) {
        if (obs != null) {
            this.pendingObs = obs;
        }
    }

    public void setObservation(Obsd[] obsData, int n) {
        if (obsData != null && n > 0) {
            this.pendingObs = new Obs();
            this.pendingObs.n = Math.min(n, obsData.length);
            for (int i = 0; i < this.pendingObs.n; i++) {
                this.pendingObs.data[i] = new Obsd(obsData[i]);
            }
        }
    }

    public Nav getNav() {
        return nav;
    }

    public Rtk getRtk() {
        return rtk;
    }

    public void resetState() {
        this.rtk = new Rtk();
        this.rtk.opt = prcOpt;
        this.lastSol = new Sol();
        this.lastNumSats = 0;
        this.lastGdop = 0.0;
    }

    private void addEphToNav(Eph eph) {
        if (nav.n < nav.nmax) {
            nav.eph[nav.n] = new Eph(eph);
            nav.n++;
        }
    }

    private void addGephToNav(Geph geph) {
        if (nav.ng < nav.ngmax) {
            nav.geph[nav.ng] = new Geph(geph);
            nav.ng++;
        }
    }

    private Obsd[] getObsData() {
        if (pendingObs != null && pendingObs.n > 0) {
            Obsd[] result = new Obsd[pendingObs.n];
            System.arraycopy(pendingObs.data, 0, result, 0, pendingObs.n);
            return result;
        }
        return null;
    }

    private void copyRtkSolToContract(Sol rtkSol, GnssPositionSolution contractSol) {
        contractSol.setTime(toContractGTime(rtkSol.time));
        contractSol.setPosEcef(new double[]{rtkSol.rr[0], rtkSol.rr[1], rtkSol.rr[2]});
        contractSol.setVelEcef(new double[]{rtkSol.rr[3], rtkSol.rr[4], rtkSol.rr[5]});
        contractSol.setSolStatus(mapSolStatus(rtkSol.stat));
        contractSol.setNumSat(rtkSol.ns);
        contractSol.setAge(rtkSol.age);
        contractSol.setRatio(rtkSol.ratio);

        double[] posCov = new double[9];
        if (rtkSol.qr != null && rtkSol.qr.length >= 6) {
            posCov[0] = rtkSol.qr[0];
            posCov[4] = rtkSol.qr[1];
            posCov[8] = rtkSol.qr[2];
            posCov[1] = posCov[3] = rtkSol.qr[3];
            posCov[5] = posCov[7] = rtkSol.qr[4];
            posCov[2] = posCov[6] = rtkSol.qr[5];
        }
        contractSol.setPosCov(posCov);

        double[] velCov = new double[9];
        if (rtkSol.qv != null && rtkSol.qv.length >= 6) {
            velCov[0] = rtkSol.qv[0];
            velCov[4] = rtkSol.qv[1];
            velCov[8] = rtkSol.qv[2];
            velCov[1] = velCov[3] = rtkSol.qv[3];
            velCov[5] = velCov[7] = rtkSol.qv[4];
            velCov[2] = velCov[6] = rtkSol.qv[5];
        }
        contractSol.setVelCov(velCov);
    }

    private GnssPositionSolution.SolutionStatus mapSolStatus(int stat) {
        switch (stat) {
            case Constants.SOLQ_FIX: return GnssPositionSolution.SolutionStatus.FIX;
            case Constants.SOLQ_FLOAT: return GnssPositionSolution.SolutionStatus.FLOAT;
            case Constants.SOLQ_SINGLE: return GnssPositionSolution.SolutionStatus.SPP;
            case Constants.SOLQ_DGPS: return GnssPositionSolution.SolutionStatus.DGPS;
            case Constants.SOLQ_PPP: return GnssPositionSolution.SolutionStatus.PPP;
            default: return GnssPositionSolution.SolutionStatus.NONE;
        }
    }

    private GTime toContractGTime(org.rtklib.java.data.GTime rtkTime) {
        return new GTime(rtkTime.time, rtkTime.sec);
    }

    private org.rtklib.java.data.GTime toRtkGTime(GTime contractTime) {
        return new org.rtklib.java.data.GTime(contractTime.time, contractTime.sec);
    }

    private PrcOpt createDefaultPrcOpt() {
        PrcOpt opt = new PrcOpt();
        opt.mode = Constants.PMODE_SINGLE;
        opt.navsys = Constants.SYS_GPS | Constants.SYS_GLO | Constants.SYS_GAL | Constants.SYS_CMP;
        opt.elmin = Math.toRadians(15.0);
        opt.nf = 2;
        opt.ionoopt = Constants.IONOOPT_BRDC;
        opt.tropopt = Constants.TROPOPT_SAAS;
        return opt;
    }

    private PrcOpt createPrcOptFromConfig(GnssConfig cfg) {
        PrcOpt opt = new PrcOpt();

        switch (cfg.getFusionMode()) {
            case TC:
                opt.mode = Constants.PMODE_KINEMA;
                opt.modear = Constants.ARMODE_FIXHOLD;
                break;
            case STC:
                opt.mode = Constants.PMODE_KINEMA;
                opt.modear = Constants.ARMODE_FIXHOLD;
                break;
            case LC:
            default:
                opt.mode = Constants.PMODE_SINGLE;
                break;
        }

        opt.navsys = Constants.SYS_GPS | Constants.SYS_GLO | Constants.SYS_GAL | Constants.SYS_CMP;
        opt.elmin = Math.toRadians(15.0);
        opt.nf = 2;
        opt.ionoopt = Constants.IONOOPT_BRDC;
        opt.tropopt = Constants.TROPOPT_SAAS;
        opt.std[0] = cfg.getPosMeasurementNoise();
        opt.std[1] = cfg.getVelMeasurementNoise();
        opt.maxinno[0] = cfg.getMaxPositionInnovation();
        opt.maxinno[1] = cfg.getMaxVelocityInnovation();
        opt.maxtdiff = cfg.getMaxUpdateTimeInterval();

        return opt;
    }

    private static double geodist(double[] rs, double[] rr, double[] e) {
        double[] r = new double[3];
        for (int i = 0; i < 3; i++) r[i] = rs[i] - rr[i];
        double d = CoordTransform.norm3(r);
        if (d <= 0.0) return 0.0;
        for (int i = 0; i < 3; i++) e[i] = r[i] / d;
        return d + Constants.OMGE / Constants.CLIGHT * (rs[0] * rr[1] - rs[1] * rr[0]);
    }

    private static double satazel(double[] pos, double[] e, double[] azel) {
        if (pos[2] < -1e6) return 0.0;
        double sinp = Math.sin(pos[0]);
        double cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]);
        double cosl = Math.cos(pos[1]);

        double east = -sinl * e[0] + cosl * e[1];
        double north = -sinp * cosl * e[0] - sinp * sinl * e[1] + cosp * e[2];
        double up = cosp * cosl * e[0] + cosp * sinl * e[1] + sinp * e[2];

        double az = Math.atan2(east, north);
        double el = Math.asin(up);

        if (azel != null && azel.length >= 2) {
            azel[0] = az < 0.0 ? az + 2.0 * Math.PI : az;
            azel[1] = el;
        }
        return el;
    }
}