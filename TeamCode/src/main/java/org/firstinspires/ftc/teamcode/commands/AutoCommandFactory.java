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
         rMove(15,0),
         forward(27, 0),
         new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 45),
         new ParallelCommandGroup(new ArmPositionCommand(armSubsystem, ArmPosition.OUT),
                 new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry)
         ).withTimeout(3000 ),
         forward(7, 45),
         new UnInstantCommand(()->clawSubsystem.close()),
         backUp(7, 45),
         new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry),
         new ArmPositionCommand(armSubsystem, ArmPosition.IN),
         new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90),
         lMove(3, -90)


        );
    }

    public Command scoreRightSample(){

        ElevatorPosCommand elevatorStartTwo = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry);
        EncoderDriveCommand forwardTwo = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 10);
        Command fullCloseClawTwo = new UnInstantCommand(()->clawSubsystem.open());
        Command BackFromSample = backUp(10, -90);
        TurnToHeadingCommand turnThree = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 45);
        ArmPositionCommand armUpTwo = new ArmPositionCommand(armSubsystem, ArmPosition.OUT);
        ElevatorPosCommand elevatorHighBasketTwo = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry);
        EncoderDriveCommand miniForwardTwo = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 7);
        Command fullOpenClawTwo = new UnInstantCommand(()->clawSubsystem.close());
        EncoderDriveCommand miniBackTwo = new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0,7);
        ElevatorPosCommand elevatorDownTwo = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry);
        ArmPositionCommand armDownTwo = new ArmPositionCommand(armSubsystem, ArmPosition.IN);
        TurnToHeadingCommand turnFour = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 0);
        EncoderDriveCommand strafeFour = new EncoderDriveCommand(mecanumDriveSubsystem, 0.0, 0.0, -.21, 8);

        return new SequentialCommandGroup(elevatorStartTwo, forwardTwo, fullCloseClawTwo, new WaitCommand(300), BackFromSample, turnThree, armUpTwo.withTimeout(200), elevatorHighBasketTwo.withTimeout(3000), miniForwardTwo, fullOpenClawTwo, miniBackTwo, elevatorDownTwo, armDownTwo, turnFour, strafeFour);
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

