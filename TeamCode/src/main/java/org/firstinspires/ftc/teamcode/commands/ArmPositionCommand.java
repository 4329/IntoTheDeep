package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.util.ArmPosition;

public class ArmPositionCommand extends CommandBase {


    private final ArmPosition armPosition;
    private final ArmSubsystem armSubsystem;

    private ArmPositionCommand(ArmSubsystem armSubsystem, ArmPosition armPosition) {


    this.armPosition = armPosition;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    }
    public static Command createCommand(ArmSubsystem armSubsystem, ArmPosition armPosition){
        return new ArmPositionCommand(armSubsystem, armPosition).withTimeout(750);

    }

    @Override
    public void initialize() {
        armSubsystem.goToPosition(armPosition);
    }


    @Override
    public void end(boolean interrupted) {

        armSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.armAtPosition();
    }
}