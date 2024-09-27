package org.firstinspires.ftc.teamcode.util;

public enum TatOrTotPosition {
    STARTTOT(0),TOTONE(30),TOTTWO(100);

    private final int value;

    TatOrTotPosition(int i) {
        this.value=i;
    }

    public int getValue() {
        return value;
    }
}
