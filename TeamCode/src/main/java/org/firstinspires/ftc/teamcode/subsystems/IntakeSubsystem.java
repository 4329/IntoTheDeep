package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private Servo servo;
    private Telemetry telemetry;
    private double position = 0;
    private double CLAW_SPEED = 0.05;
    private boolean closed = false;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.servo = hardwareMap.get(Servo.class, "intakeServo");
    }

    @Override
    public void periodic() {
        servo.setPosition(position);
        telemetry.addLine("servo position is:" + servo.getPosition());
    }
public void intake(){
            position += CLAW_SPEED;
        }

public void opener() {
            position -= CLAW_SPEED;

}

}

