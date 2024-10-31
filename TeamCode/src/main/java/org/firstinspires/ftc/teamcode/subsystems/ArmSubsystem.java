package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.FrcPidController;

public class ArmSubsystem extends SubsystemBase {
    private Motor armMotor;
    private FrcPidController pidController;
    private Telemetry telemetry;
    private int setPoint;
    private final int manuelRateOfChangeForArmRotateThing = 1;

    public ArmSubsystem(HardwareMap hm, Telemetry jerry) {
        this.armMotor = new Motor(hm, "armMotor");
        this.armMotor.setInverted(false);
        this.telemetry = jerry;
        this.pidController = new FrcPidController(.005, 0, .000005);
        this.pidController.setTolerance(2);
        this.armMotor.setRunMode(Motor.RunMode.RawPower);
        this.armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
/*        this.armMotor.setPositionCoefficient(0.05);
        this.armMotor.setPositionTolerance(0.5); */
        this.setPoint = 0;
        this.armMotor.encoder.reset();
    }

    public void goToPosition(ArmPosition timmy) {
        this.setPoint = timmy.getPosition();
        this.pidController.setSetpoint(setPoint);
    }

    @Override
    public void periodic() {
        double output = pidController.calculate(this.armMotor.encoder.getPosition());

        Log.i("ARM PID", String.format("output: %.3f, setpoint: %d, position: %d", output, setPoint, armMotor.encoder.getPosition()));
        this.armMotor.set(output);
        /*
        if (this.setPoint == ArmPosition.HORIZONTAL.getPosition()) {
            this.armMotor.set(0.0125);
        }
        else {
            this.armMotor.set(0.025);
        }
         */
        telemetry.addLine("Arm actual position: " + armMotor.getCurrentPosition());
        telemetry.addLine("Arm setPoint: " + setPoint);
        telemetry.addLine("armError: " + Math.abs(setPoint - armMotor.getCurrentPosition()));
    }

    public boolean armAtPosition() {
        return pidController.atSetpoint();

        //return armMotor.atTargetPosition();
    }

    public void armUp() {
        if (setPoint < ArmPosition.HORIZONTAL.getPosition() + manuelRateOfChangeForArmRotateThing) {
            setPoint += manuelRateOfChangeForArmRotateThing;
            pidController.setSetpoint(setPoint);
        }
        Log.i("ARM", "UP setpoint is: " + setPoint);
    }

    public void armDown() {
        if (setPoint > ArmPosition.IN.getPosition() - manuelRateOfChangeForArmRotateThing) {
            setPoint -= manuelRateOfChangeForArmRotateThing;
            pidController.setSetpoint(setPoint);
        }
        Log.i("ARM", "DOWN setpoint is: " + setPoint);
    }
    public void armStop() {
        armMotor.set(0);
    }
    public void stop() {
        this.armMotor.stopMotor();
    }
}