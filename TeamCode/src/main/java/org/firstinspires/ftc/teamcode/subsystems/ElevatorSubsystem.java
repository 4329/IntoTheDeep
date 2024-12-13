package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
    private int setPoint;
    private Motor elevatorMotor;
    private Telemetry telemetry;
    private TouchSensor elevatorSensor;
    private boolean positionControl = true;
    private boolean zeroed= false;
    private int zeroOffset = 1150;
    private final int ELEVATORMAX = 6300;

    private static final double DISTANCEPERPULSE = 0.009335691828994;

    public ElevatorSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.elevatorMotor = new Motor(hm, "elevatorMotor");
        this.elevatorSensor = hm.get(TouchSensor.class, "elevatorSensor");
        switchPosition();
    }

    public void goToPosition(ElevatorPosition sammy) {
        this.setPoint = sammy.getPosition() + (zeroed ? zeroOffset : 0);
        this.elevatorMotor.setTargetPosition(this.setPoint);




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
           Log.d("ELEVATOR", String.format("elev setpoint, pos, power, pos control, atTarget? %d, %d, %f, %s, %s", setPoint, elevatorMotor.getCurrentPosition(), elevatorMotor.get(), positionControl + "", elevatorMotor.atTargetPosition() + ""));
           this.elevatorMotor.set(1);
//            this.elevatorMotor.set(0.1);

        }
//        else {
//           this.elevatorMotor.set(0.0);
//           Log.d("ELEVATOR", String.format("NO POWER ON MOTOR elev setpoint, pos, power, pos control, pressed? %d, %d, %f, %s, %s", setPoint, elevatorMotor.getCurrentPosition(), elevatorMotor.get(), positionControl + "", elevatorSensor.isPressed() + ""));
//        }

        telemetry.addLine("Elevator setPoint:" + setPoint);
        telemetry.addLine("Elevator is at:" + elevatorMotor.getCurrentPosition());

    }

    public void stop() {
        this.elevatorMotor.stopMotor();
    }

    public void setZero() {
        setPoint =0;
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
        this.elevatorMotor.setTargetPosition(setPoint);
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

        return elevatorMotor.atTargetPosition();

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
        this.elevatorMotor.setRunMode(Motor.RunMode.RawPower);
        this.positionControl = false;
    }

    public void switchPosition() {

        this.elevatorMotor.setDistancePerPulse(DISTANCEPERPULSE);
        this.elevatorMotor.setInverted(true);
        this.elevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        this.setPoint = 0;
        this.elevatorMotor.setTargetPosition(0);
        this.elevatorMotor.setPositionCoefficient(1);
        this.elevatorMotor.setFeedforwardCoefficients(0,0.35);
//        this.elevatorMotor.setFeedforwardCoefficients(0,0.0);
        this.elevatorMotor.setPositionTolerance(5);
        this.elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.elevatorMotor.encoder.reset();
        this.positionControl = true;
    }


}