package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.util.ArmPosition;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
        super(new ParallelCommandGroup(
                new ArmPositionCommand(armSubsystem, ArmPosition.SCORE).withTimeout(200),
                new UnInstantCommand(() -> clawSubsystem.open())),
              new WaitCommand(400),
              new ArmPositionCommand(armSubsystem, ArmPosition.OUT).withTimeout(200)
         );
    }
}
