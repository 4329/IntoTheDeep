package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class TotalZeroCommand extends ParallelCommandGroup {

    public TotalZeroCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, Telemetry telemetry) {
        super(
            new ElevatorResetCommand(elevatorSubsystem, telemetry),
            new UnInstantCommand(()-> clawSubsystem.open())
        );
    }
}
