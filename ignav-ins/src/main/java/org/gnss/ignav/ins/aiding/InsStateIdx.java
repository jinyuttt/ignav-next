package org.gnss.ignav.ins.aiding;

import org.gnss.ignav.ins.data.InsOpt;

public final class InsStateIdx {

    private InsStateIdx() {}

    public static int xiA(InsOpt opt) { return 0; }
    public static int xnA(InsOpt opt) { return 3; }
    public static int xiV(InsOpt opt) { return 3; }
    public static int xnV(InsOpt opt) { return 3; }
    public static int xiP(InsOpt opt) { return 6; }
    public static int xnP(InsOpt opt) { return 3; }
    public static int xiBg(InsOpt opt) { return 9; }
    public static int xnBg(InsOpt opt) { return opt.bgopt != 0 ? 3 : 0; }
    public static int xiBa(InsOpt opt) { return 9 + (opt.bgopt != 0 ? 3 : 0); }
    public static int xnBa(InsOpt opt) { return opt.baopt != 0 ? 3 : 0; }
    public static int xiSg(InsOpt opt) { return 9 + (opt.bgopt != 0 ? 3 : 0) + (opt.baopt != 0 ? 3 : 0); }
    public static int xnSg(InsOpt opt) { return opt.estsg != 0 ? 3 : 0; }
    public static int xiSa(InsOpt opt) { return xiSg(opt) + (opt.estsg != 0 ? 3 : 0); }
    public static int xnSa(InsOpt opt) { return opt.estsa != 0 ? 3 : 0; }
    public static int xiRg(InsOpt opt) { return xiSa(opt) + (opt.estsa != 0 ? 3 : 0); }
    public static int xnRg(InsOpt opt) { return opt.estrg != 0 ? 3 : 0; }
    public static int xiRa(InsOpt opt) { return xiRg(opt) + (opt.estrg != 0 ? 3 : 0); }
    public static int xnRa(InsOpt opt) { return opt.estra != 0 ? 3 : 0; }
    public static int xiDt(InsOpt opt) { return xiRa(opt) + (opt.estra != 0 ? 3 : 0); }
    public static int xnDt(InsOpt opt) { return opt.estdt != 0 ? 1 : 0; }
    public static int xiLever(InsOpt opt) { return xiDt(opt) + (opt.estdt != 0 ? 1 : 0); }
    public static int xnLever(InsOpt opt) { return opt.estlever != 0 ? 3 : 0; }
    public static int xiOs(InsOpt opt) { return xiLever(opt) + (opt.estlever != 0 ? 3 : 0); }
    public static int xnOs(InsOpt opt) { return opt.estodos != 0 ? 1 : 0; }
    public static int xiOl(InsOpt opt) { return xiOs(opt) + (opt.estodos != 0 ? 1 : 0); }
    public static int xnOl(InsOpt opt) { return opt.estodol != 0 ? 3 : 0; }
    public static int xiOa(InsOpt opt) { return xiOl(opt) + (opt.estodol != 0 ? 3 : 0); }
    public static int xnOa(InsOpt opt) { return opt.estodoa != 0 ? 3 : 0; }

    public static int nx(InsOpt opt) {
        return xiOa(opt) + (opt.estodoa != 0 ? 3 : 0);
    }
}