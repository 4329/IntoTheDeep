package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.RobotConfig.CLAW_SPEED;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem extends SubsystemBase {

    private Telemetry telemetry;
    private Servo servo;
    private boolean closed = true;
    private double position = 0;

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.servo = hardwareMap.get(Servo.class, "clawServo");
        open();


    }


    @Override
    public void periodic() {
        servo.setPosition(position);
        telemetry.addLine("claw is: " + position);
    }

    public void open() {
        position = 0;
        closed = false;

        telemetry.addLine("claw open");

    }
    public void close() {
        position = 0.40;
        closed = true;

        telemetry.addLine("claw closed");

    }

    public void closer() {
        if (position <0.40){
            position += CLAW_SPEED;
        }
        else {
            position = 0.40;
            closed = true;
        }
    }

    public void opener() {
        if (position > 0){
            position -= CLAW_SPEED;
        }
        else {
            position = 0;
            closed = false;
        }
    }


}
