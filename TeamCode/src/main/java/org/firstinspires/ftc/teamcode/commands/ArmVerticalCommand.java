package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ArmVerticalCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private  Telemetry telemetry;
    private DoubleSupplier armPower;
    public ArmVerticalCommand(ArmSubsystem armSubsystem, DoubleSupplier armPower, Telemetry telemetry) {

        this.armPower = armPower;
this.armSubsystem = armSubsystem;
this.telemetry = telemetry;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.move(armPower.getAsDouble());
    }




//    @Override
//    public void end(boolean interrupted) {
//
//        armSubsystem.stop();
//
//    }
}