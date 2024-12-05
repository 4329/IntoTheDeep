package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ClawCloserCommand extends CommandBase {
    private ClawSubsystem clawSubsystem;

    public ClawCloserCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        clawSubsystem.closer();
    }
}
