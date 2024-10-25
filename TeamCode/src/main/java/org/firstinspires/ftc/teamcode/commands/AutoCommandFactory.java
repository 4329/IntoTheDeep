package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private TelemetryUpdateSubsystem telemetryUpdateSubsystem;
    private ImuSubsystem imuSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ClawSubsystem clawSubsystem;
    private ArmSubsystem armSubsystem;
    private Telemetry telemetry;


    public AutoCommandFactory(MecanumDriveSubsystem mecanumDriveSubsystem, TelemetryUpdateSubsystem telemetryUpdateSubsystem, ImuSubsystem imuSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem, ArmSubsystem armSubsystem, Telemetry telemetry) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.telemetryUpdateSubsystem = telemetryUpdateSubsystem;
        this.imuSubsystem = imuSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.armSubsystem = armSubsystem;
        this.telemetry = telemetry;

    }
    public Command scoreHighBasketTaterTwo () {
        return new SequentialCommandGroup(
         new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry),
                new UnInstantCommand(()->clawSubsystem.close()),
         rMove(15,0),
         forward(27, 0),
         new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 45),
         new ParallelCommandGroup(new ArmPositionCommand(armSubsystem, ArmPosition.OUT),
                 new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry)
         ).withTimeout(3000 ),
         forward(7, 45),
         new UnInstantCommand(()->clawSubsystem.open()),
         backUp(7, 45),
                new ParallelCommandGroup(
         new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry),
         new ArmPositionCommand(armSubsystem, ArmPosition.IN)
                ).withTimeout(2500),
         new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90),
         lMove(4, -90)
        );
    }


    public Command scoreRightSample (){
        return new SequentialCommandGroup (
        new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry),
        forward (10, -90),
        new UnInstantCommand(()->clawSubsystem.close()),
        new WaitCommand (500),
        backUp(10, -90),
        new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 45),
        new ParallelCommandGroup(
        new ArmPositionCommand(armSubsystem, ArmPosition.OUT),
        new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry)
                ).withTimeout(3000),
        forward(7, 45),
        new UnInstantCommand(()->clawSubsystem.open()),
        backUp (7, 45),
        new ParallelCommandGroup(
                new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry),
                new ArmPositionCommand(armSubsystem, ArmPosition.IN)
        ).withTimeout(2500),
        new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 180)
        );
    }

    public Command touchLowBar (){
        return new SequentialCommandGroup (
        new ArmPositionCommand(armSubsystem,ArmPosition.OUT).withTimeout(250),
        lMove(20,180),
        forward(8,180)
        );
    }
    private Command backUp (double inches, double heading){
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, 0.35, 0, 0,inches);
    }
    private Command forward (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, -0.35, 0, 0, inches);
    }
    private Command lMove (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, 0, 0, -.21, inches);
    }
    private Command rMove (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, 0 , 0, .21, inches);
    }
}

