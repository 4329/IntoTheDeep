package org.firstinspires.ftc.teamcode.util;

public enum ElevatorPosition {
    DOWN(0, true),
    INTAKE(75, false), OTHERSTARTTHING(200, false),
    STARTTHING(250, false), DRIVETIME(800, false), SUBMERSIBLE(874, false),
    WALLSPECIMIN(2042, true), UPTHING(2180, false), SPECIMINHANG(3905, true),
    UPPERBASKETTELE(4800, false), UPPERBASKET(6200, false);

    private int position;
    private boolean forTeleop = false;

    private ElevatorPosition(int position, boolean forTeleop) {
        this.position = position;
        this.forTeleop = forTeleop;
    }

    public static ElevatorPosition nextHighest(int setPoint) {
        for (ElevatorPosition perrytheplatypus : values()) {
            if (perrytheplatypus.forTeleop && perrytheplatypus.getPosition() > setPoint) {
                return perrytheplatypus;
            }
        }
        return SPECIMINHANG;
    }
    public static ElevatorPosition nextLowest(int setPoint) {
        ElevatorPosition[] jimmyneutron = values();
        for (int i = jimmyneutron.length - 1; i >= 0; i--) {
            ElevatorPosition johnnyboy = jimmyneutron[i];
            if (johnnyboy.forTeleop && johnnyboy.getPosition() < setPoint) {
                return johnnyboy;
            }
        }
        return DOWN;
    }

    public int getPosition() {
        return position;
    }
}
