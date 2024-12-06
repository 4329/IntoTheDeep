package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;

public class PickUpSample extends ParallelCommandGroup {
    public PickUpSample(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, Telemetry telemetry) {
        super(ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN),
        new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.INTAKE, telemetry),
                 new InstantCommand(()->clawSubsystem.open()));
    }
}
