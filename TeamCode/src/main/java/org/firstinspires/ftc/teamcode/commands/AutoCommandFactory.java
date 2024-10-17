package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;

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

    public Command scoreHighBasket(){

        ElevatorPosCommand elevatorStart = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry);

        EncoderDriveCommand strafe = new EncoderDriveCommand(mecanumDriveSubsystem, 0.0, 0.0, .21, 15);
        EncoderDriveCommand forward = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 27);
        TurnToHeadingCommand turn = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 45);
        ArmPositionCommand armUp = new ArmPositionCommand(armSubsystem, ArmPosition.OUT);
        ElevatorPosCommand elevatorHighBasket = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry);
        EncoderDriveCommand miniForward = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 7);
        Command fullOpenClaw = new UnInstantCommand(()->clawSubsystem.close());
        EncoderDriveCommand miniBack = new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0,7);
        ElevatorPosCommand elevatorDown = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry);
        ArmPositionCommand armDown = new ArmPositionCommand(armSubsystem, ArmPosition.IN);
        TurnToHeadingCommand turnTwo = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90);
        EncoderDriveCommand strafeTwo = new EncoderDriveCommand(mecanumDriveSubsystem, 0.0, 0.0, -.21, 5);

        return new SequentialCommandGroup(elevatorStart, strafe, forward, turn, armUp.withTimeout(1000), elevatorHighBasket.withTimeout(3000), miniForward.withTimeout(1000), fullOpenClaw, miniBack, elevatorDown, armDown, turnTwo, strafeTwo);



    }

    public Command scoreRightSample(){

        ElevatorPosCommand elevatorStartTwo = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry);
        EncoderDriveCommand forwardTwo = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 8);
        Command fullCloseClawTwo = new UnInstantCommand(()->clawSubsystem.open());
        EncoderDriveCommand strafeThree = new EncoderDriveCommand(mecanumDriveSubsystem, 0.0, 0.0, .21, -5);
        TurnToHeadingCommand turnThree = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 135);
        ArmPositionCommand armUpTwo = new ArmPositionCommand(armSubsystem, ArmPosition.OUT);
        ElevatorPosCommand elevatorHighBasketTwo = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry);
        EncoderDriveCommand miniForwardTwo = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 7);
        Command fullOpenClawTwo = new UnInstantCommand(()->clawSubsystem.close());
        EncoderDriveCommand miniBackTwo = new EncoderDriveCommand(mecanumDriveSubsystem, 0.35, 0, 0,7);
        ElevatorPosCommand elevatorDownTwo = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.STARTTHING, telemetry);
        ArmPositionCommand armDownTwo = new ArmPositionCommand(armSubsystem, ArmPosition.IN);
        TurnToHeadingCommand turnFour = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 0);
        EncoderDriveCommand strafeFour = new EncoderDriveCommand(mecanumDriveSubsystem, 0.0, 0.0, -.21, 8);

        return new SequentialCommandGroup(elevatorStartTwo, forwardTwo, fullCloseClawTwo, strafeThree, turnThree, armUpTwo, elevatorHighBasketTwo, miniForwardTwo, fullOpenClawTwo, miniBackTwo, elevatorDownTwo, armDownTwo, turnFour, strafeFour);
    }
}
