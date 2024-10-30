package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmUpCommand extends CommandBase {

   private final ArmSubsystem armSubsystem;

   public ArmUpCommand(ArmSubsystem armSubsystem) {
       this.armSubsystem = armSubsystem;
   }


    @Override
    public void execute() {
       armSubsystem.armUp();
    }

    @Override
    public void end(boolean interrupted) {
       armSubsystem.stop();
    }

}
