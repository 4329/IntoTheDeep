package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;

public class SubmersibleCommand extends ParallelCommandGroup {
    public SubmersibleCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, Telemetry telemetry) {
        super(new ArmPositionCommand(armSubsystem, ArmPosition.SUBMERSIBLE),
        new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.SUBMERSIBLE, telemetry),
                 new InstantCommand(()->clawSubsystem.open()));
    }
}
