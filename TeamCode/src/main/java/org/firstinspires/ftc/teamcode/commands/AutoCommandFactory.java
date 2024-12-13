package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;

import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.ArmPosition;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;

public class AutoCommandFactory {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final ImuSubsystem imuSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final ArmSubsystem armSubsystem;
    private final Telemetry telemetry;

    public AutoCommandFactory(MecanumDriveSubsystem mecanumDriveSubsystem, ImuSubsystem imuSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, ArmSubsystem armSubsystem, Telemetry telemetry) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.imuSubsystem = imuSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.armSubsystem = armSubsystem;
        this.telemetry = telemetry;

    }

    public Command scoreHighBasketTaterTwo () {
        return new SequentialCommandGroup(
             new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.OTHERSTARTTHING, telemetry),
             new UnInstantCommand(()->clawSubsystem.close()),
             rMove(15,0),
             forward(11, 0),
             new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 45),
             new ParallelCommandGroup(
                 ArmPositionCommand.createCommand(armSubsystem, ArmPosition.OUT),
                 new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry)
             ).withTimeout(3000 ),
             forward(7, 45),
             new UnInstantCommand(()->clawSubsystem.open()),
             backUp(7, 45),
             new ParallelCommandGroup(
                 new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.INTAKE, telemetry), ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN)
             ).withTimeout(2500),
             new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90),
             lMove(2, -90)
        );
    }


    public Command scoreRightSample (){
        return new SequentialCommandGroup (
            new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.INTAKE, telemetry),
            forward (9, -90),
            new UnInstantCommand(()->clawSubsystem.close()),
            new WaitCommand (1000),
            backUp(10, -90),
            new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 45),
            new ParallelCommandGroup(
                ArmPositionCommand.createCommand(armSubsystem, ArmPosition.OUT),
                new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry)
            ).withTimeout(3000),
            forward(7, 50),
            new UnInstantCommand(()->clawSubsystem.open()),
            backUp (7, 45),
            new ParallelCommandGroup(
                new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.OTHERSTARTTHING, telemetry),
                ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN)
            ).withTimeout(2500),
            new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 180)
        );
    }

    public Command touchLowBar() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    ArmPositionCommand.createCommand(armSubsystem,ArmPosition.OUT).withTimeout(250),
                    new UnInstantCommand(() ->clawSubsystem.close()),
                    new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, 180, -0.4, 0,-1, 50)
                ),
                forward(2,180),
                ArmPositionCommand.createCommand(armSubsystem,ArmPosition.BARAUTO).withTimeout(250)
        );
    }

    public Command approachPiece1(){
        return new SequentialCommandGroup(
                new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.OTHERSTARTTHING, telemetry),
                forward(1, 0),
                rMove(25, 0),
                forward(21, 0),
                new UnInstantCommand(()->clawSubsystem.close()),
                new WaitUntilCommand(clawSubsystem::atSetpoint),
                backUp(5,0),
                new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 180),
                forward(10,180),
                new UnInstantCommand(()->clawSubsystem.open()),
                new WaitUntilCommand(clawSubsystem::atSetpoint)

                );
    }

    private Command backUp (double inches, double heading){
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, 0.35, 0, 0,inches);
    }
    private Command forward (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, -0.35, 0, 0, inches);
    }

    private Command lMove (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, 0, 0, -.3, inches);
    }

    private Command rMove (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, 0 , 0, .21, inches);
    }
}

