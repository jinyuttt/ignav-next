package org.gnss.ignav.contract;

import java.io.Serializable;

public class GTime implements Serializable, Comparable<GTime> {

    private static final long serialVersionUID = 1L;

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

    public boolean equals(GTime other) {
        if (other == null) return false;
        return this.time == other.time && Math.abs(this.sec - other.sec) < 1e-12;
    }

    @Override
    public int compareTo(GTime other) {
        if (this.time < other.time) return -1;
        if (this.time > other.time) return 1;
        if (this.sec < other.sec) return -1;
        if (this.sec > other.sec) return 1;
        return 0;
    }

    public double diff(GTime other) {
        return (this.time - other.time) + (this.sec - other.sec);
    }

    @Override
    public String toString() {
        return "GTime{time=" + time + ", sec=" + sec + "}";
    }
}