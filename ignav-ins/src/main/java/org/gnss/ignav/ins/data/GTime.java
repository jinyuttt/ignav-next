package org.gnss.ignav.ins.data;

public class GTime {

    public long time;
    public double sec;

    public GTime() {
        this.time = 0;
        this.sec = 0.0;
    }

    public GTime(long time, double sec) {
        this.time = time;
        this.sec = sec;
    }

    public GTime(GTime other) {
        this.time = other.time;
        this.sec = other.sec;
    }

    public static double timeDiff(GTime t1, GTime t2) {
        return (t1.time - t2.time) + (t1.sec - t2.sec);
    }

    /**
     * GPS 时间转日历时间。
     *
     * 输入：{@code t.time} 为自 1980-01-06 起的整秒（GPST），{@code t.sec} 为秒内小数部分。
     * 输出 {@code ep}：{@code [年, 月, 日, 时, 分, 秒]}，其中月为 1–12、日为 1–31、
     * 秒为带小数的实数，与 RTKLIB {@code time2epoch} 的口径一致。
     *
     * 采用 Fliegel–Van Flandern 历法算法，正确处理闰年与世纪闰年规则。
     */
    public static void time2epoch(GTime t, double[] ep) {
        long gpst0 = 315964800L;
        long unixTime = t.time + gpst0;
        long totalDays = Math.floorDiv(unixTime, 86400);
        double sod = Math.floorMod(unixTime, 86400) + t.sec;

        long z = totalDays + 719468;
        long era = (z >= 0 ? z : z - 146096) / 146097;
        long doe = z - era * 146097;
        long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
        long year = yoe + era * 400;
        long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
        long mp = (5 * doy + 2) / 153;
        long dayOfMonth = doy - (153 * mp + 2) / 5 + 1;
        long month = mp < 10 ? mp + 3 : mp - 9;
        if (month <= 2) year++;

        ep[0] = year;
        ep[1] = month;
        ep[2] = dayOfMonth;
        ep[3] = (int) (sod / 3600);
        ep[4] = (int) ((sod % 3600) / 60);
        ep[5] = sod % 60;
    }

    @Override
    public String toString() {
        return "GTime{time=" + time + ", sec=" + sec + "}";
    }
}