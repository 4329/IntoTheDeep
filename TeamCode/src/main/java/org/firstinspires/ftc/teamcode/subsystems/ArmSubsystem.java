package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArmPosition;

public class ArmSubsystem extends SubsystemBase {
    private Motor armMotor;
    private Telemetry telemetry;
    private int setPoint;

    public ArmSubsystem(HardwareMap hm, Telemetry jerry) {
        this.armMotor = new Motor(hm, "armMotor");
        this.armMotor.setInverted(false);
        this.telemetry = jerry;
        this.armMotor.setRunMode(Motor.RunMode.PositionControl);
        this.armMotor.setPositionCoefficient(0.45);
        this.armMotor.setPositionTolerance(0.5);
        this.armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void goToPosition(ArmPosition timmy) {
        this.setPoint = timmy.getPosition();
        this.armMotor.setTargetPosition(this.setPoint);
    }

    public void resetEncoder() {
        this.setPoint = 0;
        armMotor.setTargetPosition(0);
        this.armMotor.encoder.reset();
    }

    @Override
    public void periodic() {
        if (this.setPoint == 0) {
            this.armMotor.set(0.00001);
        }
        else {
            this.armMotor.set(0.1);
        }

        telemetry.addLine("Arm actual position: " + armMotor.getCurrentPosition());
        telemetry.addLine("Arm setPoint: " + setPoint);
        telemetry.addLine("armError: " + Math.abs(setPoint - armMotor.getCurrentPosition()));
        telemetry.addLine("armEncoder:" + armMotor.encoder.getPosition());
    }

    public boolean armAtPosition() {

        return armMotor.atTargetPosition();
    }



    public void move(double stickValue) {

//        setPoint += (stickValue * 5.0);

        int newSetPoint = (int) (setPoint + (-stickValue * 3.0));

        if (newSetPoint < 0) {

            newSetPoint = 0;

        } else if (newSetPoint > 138) {
            newSetPoint = 138;
        }

    setPoint = newSetPoint;
        this.armMotor.setTargetPosition(setPoint);


    }
    public void stop() {
        this.armMotor.stopMotor();
    }
}