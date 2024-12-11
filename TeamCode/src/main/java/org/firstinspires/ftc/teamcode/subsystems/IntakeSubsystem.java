package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private CRServo servo;
    private Telemetry telemetry;
    private double position = 0;
    private double CLAW_SPEED = 0.05;

    private boolean sad = false;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.servo = hardwareMap.get(CRServo.class, "intakeServo");
    }

    @Override
    public void periodic() {

        telemetry.addLine("servo position is:" + servo.getController().getServoPosition(0));
    }

    public void intake(){
        position += CLAW_SPEED;
    }

    public void opener() {
        position -= CLAW_SPEED;
    }

}

