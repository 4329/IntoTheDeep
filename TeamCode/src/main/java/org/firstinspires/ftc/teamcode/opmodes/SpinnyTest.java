package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClawCloserCommand;
import org.firstinspires.ftc.teamcode.commands.ClawOpenerCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroups;
import org.firstinspires.ftc.teamcode.commands.ElevatorVerticalCommand;
import org.firstinspires.ftc.teamcode.commands.FireZeMisslizCommand;
import org.firstinspires.ftc.teamcode.commands.InitializeNavxCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDpadCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.UnInstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.ArmPosition;

@TeleOp(name = "Spinny Test", group = "1")
public class SpinnyTest extends CommandOpMode {
    // FtcDashboard dashboard = FtcDashboard.getInstance();

    private GamepadEx operator;
    private IntakeSubsystem intakesubsystem;
     private TelemetryUpdateSubsystem telemetryUpdateSubsystem;
    @Override
    public void initialize() {
        operator = new GamepadEx(gamepad2);
        telemetryUpdateSubsystem = new TelemetryUpdateSubsystem(telemetry);
        intakesubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        operator.getGamepadButton(GamepadKeys.Button.A).whileHeld(new StartEndCommand(()->intakesubsystem.pickup(),()->intakesubsystem.hold()));
        operator.getGamepadButton(GamepadKeys.Button.B).whileHeld(new StartEndCommand(()->intakesubsystem.rejected(),()->intakesubsystem.hold()));

        register(telemetryUpdateSubsystem);

         // >->o <- bob bobber bobbest the bobbing VIII
    }
}