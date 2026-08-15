package org.gnss.ignav.ins.data;

public class Gmeas {

    public int n;
    public int nmax;
    public Gmea[] data;

    public Gmeas() {
        this.n = 0;
        this.nmax = 0;
        this.data = null;
    }

    public void alloc(int nmax) {
        this.nmax = nmax;
        this.data = new Gmea[nmax];
        for (int i = 0; i < nmax; i++) {
            data[i] = new Gmea();
        }
        this.n = 0;
    }

    public void free() {
        this.data = null;
        this.n = 0;
        this.nmax = 0;
    }

    public int addgmea(Gmea gmea) {
        if (this.n >= this.nmax) {
            int newMax = this.nmax == 0 ? 16 : this.nmax * 2;
            Gmea[] newData = new Gmea[newMax];
            for (int i = 0; i < this.n; i++) {
                newData[i] = this.data[i];
            }
            for (int i = this.n; i < newMax; i++) {
                newData[i] = new Gmea();
            }
            this.data = newData;
            this.nmax = newMax;
        }
        if (this.data[this.n] == null) {
            this.data[this.n] = new Gmea();
        }
        Gmea.copy(gmea, this.data[this.n]);
        this.n++;
        return this.n;
    }

    public static void copy(Gmeas src, Gmeas dst) {
        dst.n = src.n;
        dst.nmax = src.nmax;
        if (src.data != null) {
            dst.data = new Gmea[src.nmax];
            for (int i = 0; i < src.nmax; i++) {
                dst.data[i] = new Gmea(src.data[i]);
            }
        } else {
            dst.data = null;
        }
    }
}