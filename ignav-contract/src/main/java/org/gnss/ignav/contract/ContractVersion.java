package org.gnss.ignav.contract;

public final class ContractVersion {

    private ContractVersion() {}

    public static final int VERSION = 1;

    public static boolean isCompatible(int providerVersion) {
        return providerVersion >= 1 && providerVersion <= VERSION;
    }
}