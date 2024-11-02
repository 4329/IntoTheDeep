package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.FrcPidController;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

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
        this.pidController = new FrcPidController(RobotConfig.ARM_P, RobotConfig.ARM_I , RobotConfig.ARM_D);   // likely need lower p.
        this.pidController.setTolerance(2);
        this.pidController.setIntegratorRange(RobotConfig.ARM_I_MIN, RobotConfig.ARM_I_MAX);
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
        if (!pidController.atSetpoint()) {
            double output = pidController.calculate(this.armMotor.encoder.getPosition());
            output += (output > 0) ? RobotConfig.ARM_FF : -RobotConfig.ARM_FF;
            double clampedOutput = MathUtil.clamp(output, -.75, .3333);

            Log.i("ARM PID", String.format("output: %.3f, clamped: %.3f, setpoint: %d, position: %d", output, clampedOutput, setPoint, armMotor.encoder.getPosition()));
            this.armMotor.set(clampedOutput);
        }
        else {
            this.armMotor.set(0); // is brake mode enough?
        }
        telemetry.addLine("Arm actual position: " + armMotor.getCurrentPosition());
        telemetry.addLine("Arm setPoint: " + setPoint);
        telemetry.addLine("Arm at setPoint: " + pidController.atSetpoint());
        telemetry.addLine("armError: " + Math.abs(setPoint - armMotor.getCurrentPosition()));
        String pidString = String.format("Arm PID:( %.8f, %.8f, %.8f) FF: %.8f", pidController.getP(), pidController.getI(), pidController.getD(), RobotConfig.ARM_FF);
        telemetry.addLine(pidString);
        Log.i("ARM settings", pidString);
    }

    public void resetPidController() {
        pidController.setPID(RobotConfig.ARM_P, RobotConfig.ARM_I, RobotConfig.ARM_D);
        pidController.setIntegratorRange(RobotConfig.ARM_I_MIN, RobotConfig.ARM_I_MAX);
        pidController.setSetpoint(setPoint);
        pidController.reset();
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