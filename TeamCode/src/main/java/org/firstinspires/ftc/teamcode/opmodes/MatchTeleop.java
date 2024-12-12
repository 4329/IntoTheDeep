package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmVerticalCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCloserCommand;
import org.firstinspires.ftc.teamcode.commands.ClawOpenerCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroups;
import org.firstinspires.ftc.teamcode.commands.ElevatorVerticalCommand;
import org.firstinspires.ftc.teamcode.commands.InitializeNavxCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDpadCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.PickUpSample;
import org.firstinspires.ftc.teamcode.commands.RaiseToHighBasket;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;
import org.firstinspires.ftc.teamcode.util.ArmPosition;

@TeleOp(name = "Match Teleop", group = "1")
public class MatchTeleop extends CommandOpMode {
    // FtcDashboard dashboard = FtcDashboard.getInstance();

    private GamepadEx driver, operator;
    private ElevatorSubsystem elevatorSubsystem;
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private TelemetryUpdateSubsystem telemetryUpdateSubsystem;
    private ImuSubsystem imuSubsystem;
    private ClawSubsystem clawSubsystem;
    private ArmSubsystem armSubsystem;
    private Command totalZeroCommandGroup;
    private Command clawOpenerCommand;
    private Command clawCloserCommand;
    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
        telemetryUpdateSubsystem = new TelemetryUpdateSubsystem(telemetry);
        imuSubsystem = new ImuSubsystem(hardwareMap, telemetry);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);
        clawSubsystem = new ClawSubsystem(hardwareMap, telemetry);
        armSubsystem = new ArmSubsystem(hardwareMap, telemetry);
        totalZeroCommandGroup = CommandGroups.totalZero(armSubsystem, elevatorSubsystem, clawSubsystem, telemetry);
        MecanumDriveCommand driveMecanumCommand = new MecanumDriveCommand(mecanumDriveSubsystem,
                () -> -driver.getLeftY(),
                () -> driver.getRightX(),
                () -> driver.getLeftX(),
                () -> driver.getButton(GamepadKeys.Button.LEFT_BUMPER),
                () -> driver.getButton(GamepadKeys.Button.A),
                telemetry);

        clawOpenerCommand = new ClawOpenerCommand(clawSubsystem);
        clawCloserCommand = new ClawCloserCommand(clawSubsystem);

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileHeld(new MecanumDpadCommand(mecanumDriveSubsystem,() -> driver.getButton(GamepadKeys.Button.B),0, 1, telemetry));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new MecanumDpadCommand(mecanumDriveSubsystem,() -> driver.getButton(GamepadKeys.Button.B),1, 0, telemetry));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new MecanumDpadCommand(mecanumDriveSubsystem,() -> driver.getButton(GamepadKeys.Button.B),-1, 0, telemetry));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(new MecanumDpadCommand(mecanumDriveSubsystem,() -> driver.getButton(GamepadKeys.Button.B),0, -1, telemetry));
        ElevatorVerticalCommand elevatorVerticalCommand = new ElevatorVerticalCommand(elevatorSubsystem, () -> operator.getLeftY(), telemetry);
        ArmVerticalCommand armVerticalCommand = new ArmVerticalCommand(armSubsystem, () -> operator.getRightY(), telemetry);
        operator.getGamepadButton(GamepadKeys.Button.X).whenHeld(clawCloserCommand);
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(()-> armSubsystem.goToPosition(ArmPosition.OUT));
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(()-> armSubsystem.goToPosition(ArmPosition.IN));
        operator.getGamepadButton(GamepadKeys.Button.B).whenHeld(clawOpenerCommand);

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()-> elevatorSubsystem.levelUp());
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()-> elevatorSubsystem.levelDown());

//        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(CommandGroups.elevatorDrive(elevatorSubsystem, armSubsystem, telemetry));
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(totalZeroCommandGroup);
        operator.getGamepadButton(GamepadKeys.Button.START).whenPressed(new PickUpSample(armSubsystem, elevatorSubsystem, clawSubsystem, telemetry));
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new RaiseToHighBasket(armSubsystem, elevatorSubsystem, clawSubsystem, telemetry));
        mecanumDriveSubsystem.setDefaultCommand(driveMecanumCommand);
        elevatorSubsystem.setDefaultCommand(elevatorVerticalCommand);
        armSubsystem.setDefaultCommand(armVerticalCommand);

        register(imuSubsystem, telemetryUpdateSubsystem);

        schedule(new InitializeNavxCommand(imuSubsystem, telemetry));
    }
}
//