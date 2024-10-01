package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TaterTotSubsystem;
import org.firstinspires.ftc.teamcode.util.TatOrTotPosition;

@TeleOp(name = "TeleOp", group = "1")
public class Teleop extends CommandOpMode {
    private GamepadEx driver;
    private GamepadEx operator;
//    private MecanumDriveSubsystem mecanumDriveSubsystem;
//  private ImuSubsystem imuSubsystem;
    private TaterTotSubsystem taterTotSubsystem;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
       // mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        // imuSubsystem = new ImuSubsystem(hardwareMap, telemetry);
        taterTotSubsystem = new TaterTotSubsystem(hardwareMap);

        // Command driveCommand = new MecanumDriveCommand(mecanumDriveSubsystem,
        //   ()-> driver.getLeftY(),
        // () -> driver.getLeftX(),
        //() -> driver.getRightX(),
        //() -> driver.getButton(GamepadKeys.Button.LEFT_BUMPER),
        //() -> driver.getButton(GamepadKeys.Button.A));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() ->taterTotSubsystem.totTater(TatOrTotPosition.TOTTWO));
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() ->taterTotSubsystem.totTater(TatOrTotPosition.TOTONE));
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(() ->taterTotSubsystem.totTater(TatOrTotPosition.STARTTOT));
        //  mecanumDriveSubsystem.setDefaultCommand(driveCommand);

        //  register(imuSubsystem);
    }
}
