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
        EncoderDriveCommand strafe = new EncoderDriveCommand(mecanumDriveSubsystem, 0.0, 0.0, .21, 10);
        EncoderDriveCommand forward = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 19);
        TurnToHeadingCommand turn = new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 45);
        ArmPositionCommand armUp = new ArmPositionCommand(armSubsystem, ArmPosition.OUT);
        ElevatorPosCommand elevatorHighBasket = new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry);
        EncoderDriveCommand miniForward = new EncoderDriveCommand(mecanumDriveSubsystem, -0.35, 0, 0, 3);
        Command fullOpenClaw = new UnInstantCommand(()->clawSubsystem.open());

        return new SequentialCommandGroup(strafe, forward, turn, armUp, elevatorHighBasket, miniForward, fullOpenClaw);


    }
}
