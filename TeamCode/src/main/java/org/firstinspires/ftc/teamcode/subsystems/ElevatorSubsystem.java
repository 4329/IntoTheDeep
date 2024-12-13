package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.RobotConfig.ELEV_D;
import static org.firstinspires.ftc.teamcode.util.RobotConfig.ELEV_I;
import static org.firstinspires.ftc.teamcode.util.RobotConfig.ELEV_I_MAX;
import static org.firstinspires.ftc.teamcode.util.RobotConfig.ELEV_I_MIN;
import static org.firstinspires.ftc.teamcode.util.RobotConfig.ELEV_P;
import static org.firstinspires.ftc.teamcode.util.RobotConfig.ELEV_TOLERANCE;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;
import org.firstinspires.ftc.teamcode.util.FrcPidController;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

public class ElevatorSubsystem extends SubsystemBase {
    private int setPoint;
    private Motor elevatorMotor;
    private Telemetry telemetry;
    private TouchSensor elevatorSensor;
    private FrcPidController pidController;
    private boolean positionControl = true;
    private boolean zeroed= false;
    private int zeroOffset = 1200;
    private final int ELEVATORMAX = 6300;

    private static final double DISTANCEPERPULSE = 0.009335691828994;

    public ElevatorSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.elevatorMotor = new Motor(hm, "elevatorMotor");
        this.elevatorSensor = hm.get(TouchSensor.class, "elevatorSensor");
        this.elevatorMotor.setRunMode(Motor.RunMode.RawPower);
        this.elevatorMotor.setInverted(true);
        this.elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.pidController = new FrcPidController(ELEV_P, ELEV_I, ELEV_D);
        switchPosition();
        resetPidController();
    }

    public void resetPidController() {
        pidController.setPID(ELEV_P, ELEV_I, ELEV_D);
        pidController.setIntegratorRange(ELEV_I_MIN, ELEV_I_MAX);
        pidController.setTolerance(ELEV_TOLERANCE);
        pidController.setSetpoint(setPoint);
        pidController.reset();
    }

    public void goToPosition(ElevatorPosition sammy) {
        this.setPoint = sammy.getPosition() + (zeroed ? zeroOffset : 0);
        pidController.setSetpoint(setPoint);
        resetPidController(); // if we change the setpoint, we should clear out our error.
        //this.elevatorMotor.setTargetPosition(this.setPoint);
    }

    public void elevatorAtZero(){
        zeroed = true;
    }

    @Override
    public void periodic() {
        if (elevatorSensor != null) {
            telemetry.addLine("the elevator is pressed?" + elevatorSensor.isPressed());
        }

        if(this.elevatorMotor!=null && positionControl){
            doPid();
        }

        telemetry.addLine("Elevator setPoint:" + setPoint);
        telemetry.addLine("Elevator is at:" + elevatorMotor.getCurrentPosition());
        telemetry.addLine(String.format("ELEV PID: (%.3f, %.3f, %.3f)", pidController.getD(), pidController.getI(), pidController.getD()));
        telemetry.addLine(String.format("ELEV tol: %.3f, IR (%.2f, %.2f), tol: ", pidController.getPositionTolerance(), ELEV_I_MIN, ELEV_I_MAX));
    }

    private void doPid() {
        if (!pidController.atSetpoint()) {
            double encPosition = elevatorMotor.encoder.getPosition();
            double output = pidController.calculate(encPosition);
            output += (output > 0) ? RobotConfig.ELEV_FF : -RobotConfig.ELEV_FF;
            double clampedOutput = MathUtil.clamp(output, -.75, .3333);

            Log.i("ELEV PID", String.format("output: %.3f, clamped: %.3f, setpoint: %d, position: %.3f", output, clampedOutput, setPoint, encPosition));
            this.elevatorMotor.set(clampedOutput);
        }
        else {
            this.elevatorMotor.set(0); // is brake mode enough?
        }
    }

    public void stop() {
        this.elevatorMotor.stopMotor();
    }

    public void move(double stickValue) {
//        setPoint += (stickValue * 5.0);
        int newSetPoint = (int) (setPoint + (stickValue * 77.0));
        if (newSetPoint < 0) {
            newSetPoint = 0;
        }
        else if (newSetPoint > ELEVATORMAX){
            newSetPoint = ELEVATORMAX;
        }
        Log.d("ELEVATOR", "manually moving to: " + newSetPoint);
        setPoint = newSetPoint;
        pidController.setSetpoint(setPoint);
        resetPidController();
        //this.elevatorMotor.setTargetPosition(setPoint);
    }

    public void levelUp() {
        ElevatorPosition up = ElevatorPosition.nextHighest(setPoint);
        goToPosition(up);
    }

    public void levelDown() {
        ElevatorPosition down = ElevatorPosition.nextLowest(setPoint);
        goToPosition(down);
    }

    public boolean uThereYet(){
        return pidController.atSetpoint();
    }

    public boolean isDown() {
        telemetry.addLine("ELEVATORISDOWN" + elevatorSensor.isPressed());
        return  elevatorSensor.isPressed();
    }

    public void goDown() {
        elevatorMotor.set(-1);
    }

    public void goUp() {
        elevatorMotor.set(0.7);
    }

    public void switchPower() {
        this.positionControl = false;
    }

    public void switchPosition() {
        this.positionControl = true;
        this.setPoint = 0;
        this.elevatorMotor.encoder.reset();
        resetPidController();
    }
}