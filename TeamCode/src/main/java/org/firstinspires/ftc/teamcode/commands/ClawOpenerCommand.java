package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ClawOpenerCommand extends CommandBase {
    private ClawSubsystem clawSubsystem;

    public ClawOpenerCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        clawSubsystem.opener();
    }
}
