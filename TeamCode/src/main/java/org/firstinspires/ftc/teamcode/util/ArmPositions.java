package org.firstinspires.ftc.teamcode.util;

public enum ArmPositions {
    ZERO(0),MAX(38531);

    private final int value;

    ArmPositions(int i) {
        this.value=i;
    }

    public int getValue() {
        return value;
    }
}