package org.firstinspires.ftc.teamcode.util;

public enum ArmPosition {
    OUT(135), IN(0), SPECIMANEHANG(40), SUBMERSIBLE(70), BARAUTO(110), FORTOTALZERO(70), SCORE(125);
    private int position;

    private ArmPosition(int position) {
        this.position = position;
    }

    public int getPosition() {
        return position;
    }
}
