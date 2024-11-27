package org.firstinspires.ftc.teamcode.util;

public enum ArmPosition {
    OUT(135), IN(0), SUBMERSIBLE(70), BARAUTO(110), FORTOTALZERO(70);
    private int position;

    private ArmPosition(int position) {
        this.position = position;
    }

    public int getPosition() {
        return position;
    }
}
