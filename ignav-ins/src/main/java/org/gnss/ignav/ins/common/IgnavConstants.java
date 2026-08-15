package org.gnss.ignav.ins.common;

public final class IgnavConstants {

    private IgnavConstants() {}

    public static final double CLIGHT = 299792458.0;
    public static final double OMGE = 7.2921151467E-5;
    public static final double RE_WGS84 = 6378137.0;
    public static final double FE_WGS84 = 1.0 / 298.257223563;
    public static final long GPST0_TIME = 315964800L;
    public static final double HION = 350000.0;

    public static final int MAXFREQ = 7;

    public static final double FREQ1 = 1.57542E9;
    public static final double FREQ2 = 1.22760E9;
    public static final double FREQ5 = 1.17645E9;
    public static final double FREQ6 = 1.27875E9;
    public static final double FREQ7 = 1.20714E9;
    public static final double FREQ8 = 1.191795E9;
    public static final double FREQ9 = 2.492028E9;
    public static final double FREQ1_GLO = 1.60200E9;
    public static final double DFRQ1_GLO = 0.56250E6;
    public static final double FREQ2_GLO = 1.24600E9;
    public static final double DFRQ2_GLO = 0.43750E6;
    public static final double FREQ3_GLO = 1.202025E9;
    public static final double FREQ1_CMP = 1.561098E9;
    public static final double FREQ2_CMP = 1.20714E9;
    public static final double FREQ3_CMP = 1.26852E9;

    public static final double EFACT_GPS = 1.0;
    public static final double EFACT_GLO = 1.5;
    public static final double EFACT_GAL = 1.0;
    public static final double EFACT_QZS = 1.0;
    public static final double EFACT_CMP = 1.0;
    public static final double EFACT_IRN = 1.5;
    public static final double EFACT_SBS = 3.0;

    public static final double D2R = Math.PI / 180.0;
    public static final double R2D = 180.0 / Math.PI;
    public static final double DEG2R = D2R / 3600.0;
    public static final double MG2M = 1E-3 * 9.7803253359;

    public static final double MU = 3.986004418E14;
    public static final double J2 = 1.082627E-3;
    public static final double WGS_E = 0.0818191908425;
    public static final double RP = 6356752.31425;
    public static final double E_SQR = 0.00669437999014;

    public static final int INS_BAOFF = 0;
    public static final int INS_BAEST = 1;
    public static final int INS_BGOFF = 0;
    public static final int INS_BGEST = 1;
    public static final int INS_RGEST = 1;
    public static final int INS_RAEST = 1;

    public static final int INSS_NONE = 0;
    public static final int INSS_INIT = 0;
    public static final int INSS_ALIGN = 1;
    public static final int INSS_MECH = 2;
    public static final int INSS_LCUD = 3;
    public static final int INSS_TCUD = 4;
    public static final int INSS_TIME = 5;
    public static final int INSS_REBOOT = 6;
    public static final int INSS_LACK = 7;
    public static final int INSS_ZVU = 8;
    public static final int INSS_ZARU = 9;
    public static final int INSS_NHC = 10;
    public static final int INSS_ODO = 11;
    public static final int INSS_RTS = 12;
    public static final int INSS_MAGH = 13;

    public static final int NPOS = 5;

    public static final int IMUDETST_GLRT = 0;
    public static final int IMUDETST_MV = 1;
    public static final int IMUDETST_MAG = 2;
    public static final int IMUDETST_ARE = 3;
    public static final int IMUDETST_ALL = 4;

    public static final int INSUPD_INSS = 0;
    public static final int INSUPD_TIME = 1;
    public static final int INSUPD_MEAS = 2;

    public static final int INS_MECH_NED = 0;
    public static final int INS_MECH_ECEF = 1;

    public static final int INS_ALIGN_COARSE = 0;
    public static final int INS_ALIGN_FINE = 1;
    public static final int INS_ALIGN_FINEEX = 2;
    public static final int INS_ALIGN_FINE_LYM = 3;
    public static final int INS_ALIGN_VEL_MATCH = 4;

    public static final int INS_LC = 0;
    public static final int INS_TC = 1;

    public static final int INS_UPDATE_OK = 0;
    public static final int INS_UPDATE_FAIL = -1;

    public static final int MAXIMU = 5;
    public static final int MAXIMUBUF = 36000;

    public static final double PI = Math.PI;
}