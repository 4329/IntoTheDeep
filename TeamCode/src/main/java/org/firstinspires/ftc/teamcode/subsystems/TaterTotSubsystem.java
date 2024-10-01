package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.TatOrTotPosition;

public class TaterTotSubsystem extends SubsystemBase {
    private int setpoint;
    private Motor tatertot;
    public TaterTotSubsystem(HardwareMap tatertoot) {
        this.tatertot = new Motor(tatertoot, "theTaterTot");
        this.tatertot.setRunMode(Motor.RunMode.PositionControl);
        this.tatertot.setPositionCoefficient(.17);
        this.tatertot.setPositionTolerance(0.5);
        this.tatertot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.tatertot.resetEncoder();
        this.setpoint=0;
    }
    public void totTater(TatOrTotPosition totOrTate){
        this.setpoint=totOrTate.getValue();
        this.tatertot.setTargetPosition(setpoint);
    }
    public void stopTots(){
        this.tatertot.stopMotor();
    }

    @Override
    public void periodic() {
        //  this.tatertot.setTargetPosition(setpoint);
        this.tatertot.set(0.1);
    }
}
