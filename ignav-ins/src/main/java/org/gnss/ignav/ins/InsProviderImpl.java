package org.gnss.ignav.ins;

import org.gnss.ignav.contract.ContractVersion;
import org.gnss.ignav.contract.ImuMeasurement;
import org.gnss.ignav.contract.InsConfig;
import org.gnss.ignav.contract.InsPrediction;
import org.gnss.ignav.contract.InsProvider;
import org.gnss.ignav.contract.InsSolution;
import org.gnss.ignav.contract.StateCorrection;
import org.gnss.ignav.contract.GTime;
import org.gnss.ignav.contract.SystemHealth;
import org.gnss.ignav.contract.TimeProvider;
import org.gnss.ignav.ins.common.IgnavConstants;
import org.gnss.ignav.ins.aiding.InsNhc;
import org.gnss.ignav.ins.aiding.InsOdo;
import org.gnss.ignav.ins.aiding.InsStateIdx;
import org.gnss.ignav.ins.aiding.InsZaru;
import org.gnss.ignav.ins.aiding.InsZvu;
import org.gnss.ignav.ins.common.InsMath;
import org.gnss.ignav.ins.data.Imud;
import org.gnss.ignav.ins.data.InsOpt;
import org.gnss.ignav.ins.data.InsState;
import org.gnss.ignav.ins.data.Odod;
import org.gnss.ignav.ins.ekf.InsEkf;
import org.gnss.ignav.ins.mech.InsAlignMech;
import org.gnss.ignav.ins.mech.InsMech;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class InsProviderImpl implements InsProvider {

    private static final Logger logger = LoggerFactory.getLogger(InsProviderImpl.class);

    private InsState insState;
    private InsOpt insOpt;
    private boolean configured = false;
    private TimeProvider timeProvider;

    public InsProviderImpl() {
        this.insState = new InsState();
        this.insOpt = new InsOpt();
    }

    @Override
    public void timeUpdate(ImuMeasurement imu) {
        if (!configured) {
            logger.warn("INS not configured, call configure() first");
            return;
        }

        Imud imud = convertImuMeasurement(imu);

        InsMech.adjustimu(insOpt, imud);

        int result;
        if (insOpt.imuformat == 0) {
            result = InsMech.updateins(insOpt, insState, imud);
        } else {
            result = InsMech.updateinsn(insOpt, insState, imud);
        }

        if (result != 0 && insState.nx > 0) {
            InsEkf.predict(insState, insOpt, insState.dt);
        }

        if (insOpt.nhc != 0) {
            InsNhc.nhc(insState, insOpt, imud);
        }
        if (insOpt.zvu != 0) {
            InsZvu.zvu(insState, insOpt, imud, 1);
        }
        if (insOpt.zaru != 0) {
            InsZaru.zaru(insState, insOpt, imud, 1);
        }
        if (insOpt.odo != 0 && imud.odoc > 0) {
            InsOdo.odo(insOpt, imud, imud.odo, insState);
        }
    }

    @Override
    public InsPrediction getPrediction() {
        InsPrediction pred = new InsPrediction();

        if (insState == null) return pred;

        pred.setTime(convertGTime(insState.time));
        pred.setPosEcef(insState.re.clone());
        pred.setVelEcef(insState.ve.clone());

        double[] q = new double[4];
        InsMech.getqbn(insState, q);
        pred.setAttQuat(q);

        if (insState.P != null && insState.nx > 0) {
            int IP = InsStateIdx.xiP(insOpt);
            int IV = InsStateIdx.xiV(insOpt);
            int IA = InsStateIdx.xiA(insOpt);
            int nx = insState.nx;

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    pred.getPosCov()[i * 3 + j] = insState.P[(IP + i) + (IP + j) * nx];
                    pred.getVelCov()[i * 3 + j] = insState.P[(IV + i) + (IV + j) * nx];
                    pred.getAttCov()[i * 3 + j] = insState.P[(IA + i) + (IA + j) * nx];
                }
            }
        }

        return pred;
    }

    @Override
    public void applyCorrection(StateCorrection correction) {
        if (correction == null || insState == null) return;

        double[] dx = correction.getDx();
        double[] dP = correction.getDP();

        if (dx != null && insState.x != null && dx.length == insState.x.length) {
            for (int i = 0; i < dx.length; i++) {
                insState.x[i] += dx[i];
            }
            InsNhc.clp(insState, insOpt, dx);
        }

        if (dP != null && insState.P != null && dP.length == insState.P.length) {
            for (int i = 0; i < dP.length; i++) {
                insState.P[i] += dP[i];
            }
        }
    }

    @Override
    public InsSolution getSolution() {
        InsSolution sol = new InsSolution();

        if (insState == null) return sol;

        sol.setTime(convertGTime(insState.time));
        sol.setPosEcef(insState.re.clone());
        sol.setVelEcef(insState.ve.clone());

        double[] q = new double[4];
        InsMech.getqbn(insState, q);
        sol.setAttQuat(q);

        if (insState.P != null && insState.nx > 0) {
            int IP = InsStateIdx.xiP(insOpt);
            int IV = InsStateIdx.xiV(insOpt);
            int IA = InsStateIdx.xiA(insOpt);
            int nx = insState.nx;

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    sol.getPosCov()[i * 3 + j] = insState.P[(IP + i) + (IP + j) * nx];
                    sol.getVelCov()[i * 3 + j] = insState.P[(IV + i) + (IV + j) * nx];
                    sol.getAttCov()[i * 3 + j] = insState.P[(IA + i) + (IA + j) * nx];
                }
            }
        }

        int status = InsSolution.STATUS_NONE;
        if (insState.stat == IgnavConstants.INSS_ALIGN) status = InsSolution.STATUS_ALIGNING;
        else if (insState.stat == IgnavConstants.INSS_MECH) status = InsSolution.STATUS_NAVIGATING;
        sol.setStatus(status);

        return sol;
    }

    @Override
    public void configure(InsConfig config) {
        if (config == null) {
            logger.warn("InsConfig is null");
            return;
        }

        insOpt.psd.gyro = config.getGyroNoisePsd();
        insOpt.psd.accl = config.getAcclNoisePsd();
        insOpt.psd.bg = config.getBgPsd();
        insOpt.psd.ba = config.getBaPsd();
        insOpt.psd.dt = config.getDtPsd();
        insOpt.psd.sg = config.getSgPsd();
        insOpt.psd.sa = config.getSaPsd();
        insOpt.psd.rg = config.getRgPsd();
        insOpt.psd.ra = config.getRaPsd();

        insOpt.baopt = config.isEstimateBa() ? 1 : 0;
        insOpt.bgopt = config.isEstimateBg() ? 1 : 0;
        insOpt.estdt = config.isEstimateDt() ? 1 : 0;
        insOpt.estsg = config.isEstimateSg() ? 1 : 0;
        insOpt.estsa = config.isEstimateSa() ? 1 : 0;
        insOpt.estrg = config.isEstimateRg() ? 1 : 0;
        insOpt.estra = config.isEstimateRa() ? 1 : 0;
        insOpt.estlever = config.isEstimateLever() ? 1 : 0;

        insOpt.nhc = config.isEnableNhc() ? 1 : 0;
        insOpt.zvu = config.isEnableZvu() ? 1 : 0;
        insOpt.zaru = config.isEnableZaru() ? 1 : 0;
        insOpt.odo = config.isEnableOdo() ? 1 : 0;

        if (config.getLeverArm() != null) {
            System.arraycopy(config.getLeverArm(), 0, insOpt.lever, 0, 3);
        }

        insOpt.odopt.s = config.getOdoScale();
        if (config.getOdoLever() != null) {
            System.arraycopy(config.getOdoLever(), 0, insOpt.odopt.lever, 0, 3);
        }

        if (config.getInitPosEcef() != null) {
            System.arraycopy(config.getInitPosEcef(), 0, insState.re, 0, 3);
        }
        if (config.getInitVelEcef() != null) {
            System.arraycopy(config.getInitVelEcef(), 0, insState.ve, 0, 3);
        }
        if (config.getInitAttQuat() != null && config.getInitAttQuat().length == 4) {
            double[] C = new double[9];
            InsMath.quat2dcm(config.getInitAttQuat(), C);
            double[] pos = new double[3];
            double[] Cne = new double[9];
            InsMath.ecef2pos(insState.re, pos);
            InsMath.ned2xyz(pos, Cne);
            InsMath.matmul3("NN", Cne, C, insState.Cbe);
        }

        insOpt.unc.att = config.getInitAttStd();
        if (config.getInitPosStd() != null) {
            insOpt.unc.pos = config.getInitPosStd()[0];
        }
        if (config.getInitVelStd() != null) {
            insOpt.unc.vel = config.getInitVelStd()[0];
        }

        InsEkf.initEkf(insState, insOpt);

        configured = true;
        logger.info("INS configured successfully, nx={}", insState.nx);
    }

    @Override
    public int getSupportedContractVersion() {
        return ContractVersion.VERSION;
    }

    @Override
    public void setTimeProvider(TimeProvider timeProvider) {
        this.timeProvider = timeProvider;
    }

    @Override
    public SystemHealth getHealth() {
        SystemHealth health = new SystemHealth();

        if (!configured) {
            health.setStatus(SystemHealth.HealthStatus.FAILED);
            health.setReason("INS not configured");
            return health;
        }

        double covTrace = 0.0;
        if (insState.P != null && insState.nx > 0) {
            int IP = InsStateIdx.xiP(insOpt);
            int IV = InsStateIdx.xiV(insOpt);
            int nx = insState.nx;
            for (int i = 0; i < 3; i++) {
                covTrace += insState.P[(IP + i) + (IP + i) * nx];
                covTrace += insState.P[(IV + i) + (IV + i) * nx];
            }
        }
        health.setInsCovTrace(covTrace);

        if (insState.stat == IgnavConstants.INSS_NONE) {
            health.setStatus(SystemHealth.HealthStatus.FAILED);
            health.setReason("INS not initialized");
        } else if (insState.stat == IgnavConstants.INSS_ALIGN) {
            health.setStatus(SystemHealth.HealthStatus.INS_DEGRADED);
            health.setReason("INS aligning");
        } else if (covTrace > 1e6) {
            health.setStatus(SystemHealth.HealthStatus.INS_DEGRADED);
            health.setReason("INS covariance too large");
        } else {
            health.setStatus(SystemHealth.HealthStatus.NOMINAL);
            health.setReason("INS operating normally");
        }

        return health;
    }

    private Imud convertImuMeasurement(ImuMeasurement imu) {
        Imud imud = new Imud();
        imud.time = new org.gnss.ignav.ins.data.GTime(
            imu.getTime().time, imu.getTime().sec);
        if (imu.getGyro() != null) {
            System.arraycopy(imu.getGyro(), 0, imud.gyro, 0, 3);
        }
        if (imu.getAccl() != null) {
            System.arraycopy(imu.getAccl(), 0, imud.accl, 0, 3);
        }
        return imud;
    }

    private GTime convertGTime(org.gnss.ignav.ins.data.GTime gt) {
        if (gt == null) return new GTime();
        return new GTime(gt.time, gt.sec);
    }
}