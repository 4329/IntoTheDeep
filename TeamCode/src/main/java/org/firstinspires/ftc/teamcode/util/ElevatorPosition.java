package org.firstinspires.ftc.teamcode.util;

public enum ElevatorPosition {

    DOWN(0),
    INTAKE(75), OTHERSTARTTHING(200), STARTTHING(250), DRIVETIME(800), SUBMERSIBLE(874),
    WALLSPECIMIN(2110), UPTHING(2180), SPECIMINHANG(4450), UPPERBASKETTELE(4800), UPPERBASKET(6200);
    private int position;

    private ElevatorPosition(int position) {
        this.position = position;
    }

    public static ElevatorPosition nextHighest(int setPoint) {
        for (ElevatorPosition perrytheplatypus : values()) {
            if (perrytheplatypus.getPosition() > setPoint) {
                return perrytheplatypus;
            }
        }
        return UPPERBASKET;
    }
    public static ElevatorPosition nextLowest(int setPoint) {
        ElevatorPosition[] jimmyneutron = values();
        for (int i = jimmyneutron.length - 1; i >= 0; i--) {
            ElevatorPosition johnnyboy = jimmyneutron[i];
            if (johnnyboy.getPosition() < setPoint) {
                return johnnyboy;
            }
        }
        return DOWN;
    }

    public int getPosition() {
        return position;
    }
}
