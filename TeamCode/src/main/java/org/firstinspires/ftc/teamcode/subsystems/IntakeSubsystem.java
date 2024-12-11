package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;

public class IntakeSubsystem extends SubsystemBase {
    private CRServo servo;
    private Telemetry telemetry;

    private boolean sad = false;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.servo = hardwareMap.get(CRServo.class, "intakeServo");
    }

    @Override
    public void periodic() {
    }

    public void intake(){
        servo.setPower(1);
    }

    public void opener() {
        servo.setPower(-1);
    }

    public void stop() {
        servo.setPower(0);
    }

}

