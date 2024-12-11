package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArmPositions;
import org.firstinspires.ftc.teamcode.util.TatOrTotPosition;

public class ArmSubsystem extends SubsystemBase {

    private int setpoint;
    private Motor armMotor;
    private Telemetry telemetry;
    private int rateOfChange = 20; //no greater than 5, 4 may be desired


    public ArmSubsystem(HardwareMap armMotor, Telemetry telemetry) {
        this.armMotor = new Motor(armMotor, "armMotor");
        //this.tatertot.setInverted(true);
        this.telemetry = telemetry;
        this.armMotor.setRunMode(Motor.RunMode.PositionControl);
        this.armMotor.setPositionCoefficient(.17);
        this.armMotor.setPositionTolerance(0.5);
        this.armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.armMotor.resetEncoder();
        this.setpoint=0;
        this.armMotor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void totTater(TatOrTotPosition totOrTate){
        this.setpoint=totOrTate.getValue();
        this.armMotor.setTargetPosition(setpoint);
    }
    public void stopArm(){
        this.armMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (telemetry != null){

            //this.telemetry.addLine("armMotor setpoint: " + setpoint);
            //this.telemetry.addLine("armMotor actual position: " + armMotor.getCurrentPosition());
            telemetry.update();
        }
        else{
            Log.i("arm", "arm is at: " + setpoint);
        }

        this.armMotor.set(0.1);
    }

    public void down() {
        if (setpoint > 0){
            setpoint = setpoint - rateOfChange;
            this.armMotor.setTargetPosition(setpoint);

        }
    }

    public void up() {
        if (setpoint < ArmPositions.MAX.getValue()){
            setpoint = setpoint + rateOfChange;
            this.armMotor.setTargetPosition(setpoint);

        }

    }
}