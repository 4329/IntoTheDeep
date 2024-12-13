package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConfig {
    public static double CLAW_SPEED = .02;

    public static double ELEV_P = 0.07;
    public static double ELEV_I = 0.005;
    public static double ELEV_D = 0.000005;
    public static double ELEV_FF = 0;
    public static double ELEV_I_MIN = -1;
    public static double ELEV_I_MAX = 1;
    public static double ELEV_TOLERANCE = 5;
}
