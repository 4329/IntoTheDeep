package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.RobotConfig.CLAW_SPEED;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {

    private Telemetry telemetry;
    private Servo servo;


    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.servo = hardwareMap.get(Servo.class, "spinnyServo");

    }


    @Override
    public void periodic() {
       // servo.setPosition(position);
        // telemetry.addLine("claw is: " + position);
    }

    public void pickup() {
     servo.setPosition(1);

        telemetry.addLine("spinny pickup");

    }
    public void rejected() {
       servo.setPosition(0);

        telemetry.addLine("spinny rejected");

    }
public void hold() {
        servo.setPosition(0.5);

        telemetry.addLine("spinny hold");

}


}
