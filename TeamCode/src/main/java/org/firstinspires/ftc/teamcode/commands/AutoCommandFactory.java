package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.dfrobot.HuskyLensSubsystem;

import org.ejml.dense.row.linsol.qr.LinearSolverQrHouseTran_ZDRM;
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
//             new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.OTHERSTARTTHING, telemetry),
             new UnInstantCommand(()->clawSubsystem.close()),
                new ParallelCommandGroup(
                ArmPositionCommand.createCommand(armSubsystem, ArmPosition.OUT),
                new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry),
             rMove(15,0)).withTimeout(3000),

             forward(11, 0),
             new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 45),
//             new ParallelCommandGroup(

//             ).withTimeout(3000 ),
             forward(5, 45),
             new ScoreCommand(armSubsystem, clawSubsystem),
//             new UnInstantCommand(()->clawSubsystem.open()),
//             new WaitCommand(750),
             backUp(6, 45),
             new ParallelCommandGroup(
                 new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.INTAKE, telemetry), ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN)
             ).withTimeout(2500),
             new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90) //,
             // lMove(2.25, -90)
                // rMove(.5, -90)

        );
    }


    public Command scoreRightSample (){
        return new SequentialCommandGroup (
            new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.INTAKE, telemetry),
            lMove(.7, -90),
                forward (14, -90),
                new WaitCommand (500),
            new UnInstantCommand(()->clawSubsystem.close()),
            new WaitCommand (1000),
          //  backUp(15, -90),
            new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 90),
            new ParallelCommandGroup(
                    forward(15, 90),
                ArmPositionCommand.createCommand(armSubsystem, ArmPosition.OUT),
                new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry)

            ).withTimeout(3000),
                new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 45),
                forward(5,45),
                new ScoreCommand(armSubsystem, clawSubsystem),
//            new UnInstantCommand(()->clawSubsystem.open()),
//            new WaitCommand(750),
            backUp (6, 45),
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
                    new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, 180, -0.3, 0,-2, 50)
                ),
                forward(3,180),
                ArmPositionCommand.createCommand(armSubsystem,(ArmPosition.BARAUTO)).withTimeout(250),
               // new WaitCommand(500),
                new UnInstantCommand(()-> armSubsystem.stop())

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
    private Command slowForward (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, -0.2, 0, 0, inches);
    }

    private Command lMove (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, 0, 0, -.3, inches);
    }
    private Command lFastMove (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, 0, 0, -.7, inches);
    }
    private Command rMove (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, 0 , 0, .21, inches);
    }
    private Command rFastMove (double inches, double heading) {
        return new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, heading, 0 , 0, .7, inches);
    }
    public Command hangHighSample() {
        return new SequentialCommandGroup(
                new UnInstantCommand(()->clawSubsystem.close()),
                new ParallelCommandGroup(
                    new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.SPECIMINHANG, telemetry),
                    ArmPositionCommand.createCommand(armSubsystem, ArmPosition.SPECIMANEHANG)),
                forward (26, 0),
                new UnInstantCommand(() ->clawSubsystem.open()),
                backUp(5, 0)
        );
    }

    public Command grabBlueSample() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DRIVETIME, telemetry),
                        ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN)),
                rMove(29, 0),
                forward(28,0),
                new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 180),
                lMove(10,180),
                forward(45, 180)
        );
    }

    public Command pickUpSampleOffWall() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.WALLSPECIMIN, telemetry),
                    ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN),
                    rMove(29, 0)),
                forward(28,0),
                new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 180),
                lMove(11,180),
                forward(50, 180),
                new WaitCommand(500),
                //--------------------------------------------------------------------------------------------------------------
                new ParallelCommandGroup(
                      new UnInstantCommand(() -> clawSubsystem.close()),
                      slowForward(50,180)).withTimeout(1000),
                new WaitCommand(500),
                new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.SPECIMINHANG, telemetry),
                backUp(10, 180),
                new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 0),
                new ParallelCommandGroup(
                    lFastMove(53, 0),
                    ArmPositionCommand.createCommand(armSubsystem, ArmPosition.SPECIMANEHANG)),
                forward(16, 0),
                new WaitCommand(200),
                new UnInstantCommand(() -> clawSubsystem.open()),
                backUp(20, 0),
                new ParallelCommandGroup(
                        new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.DOWN, telemetry),
                        rFastMove(37, 0),
                        ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN)
                ),
                backUp(2,0)
        );
    }

    public Command AdvancedBasketOne() {

        return new SequentialCommandGroup(
            new UnInstantCommand(() -> clawSubsystem.close()),
            new WaitCommand(100),
            new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new WaitCommand(600),
                            ArmPositionCommand.createCommand(armSubsystem, ArmPosition.OUT).withTimeout(1000)
                    ),
                    new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry).withTimeout(1400),
                    rMove(17, 0)),
            new ParallelCommandGroup(
                    new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry).withTimeout(200),
                    ArmPositionCommand.createCommand(armSubsystem, ArmPosition.OUT).withTimeout(200),
                    forward(4, 0)
            ),
            new WaitCommand(200),
            new ScoreCommand(armSubsystem, clawSubsystem),
            new WaitCommand(75),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN)
                ),
                new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.INTAKE, telemetry),
                backUp(14.5, 0)
            ),
            new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90).withTimeout(1000),
                forward(7, -90),
                new UnInstantCommand(()-> clawSubsystem.close()),
                new WaitCommand(500),
                new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 0).withTimeout(1000),
                new UnInstantCommand(()-> clawSubsystem.close())

        );

    }
//
    public Command AdvancedBasketTwo() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                       ArmPositionCommand.createCommand(armSubsystem, ArmPosition.OUT),
                        new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry).withTimeout(2000),
                    new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, 0, -0.95, 0, -0.65, 19)
                ),
                new WaitCommand(200),
                new ScoreCommand(armSubsystem, clawSubsystem),
                new WaitCommand(75),
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new WaitCommand(300),
                        ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN)
                    ),
                    new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.INTAKE, telemetry),
                    backUp(12.5, 0)
                ),
                new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90).withTimeout(1000),
                forward(9, -90),
                new UnInstantCommand(()-> clawSubsystem.close()),
                new WaitCommand(500),
                new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, 0).withTimeout(1000),
                new UnInstantCommand(()-> clawSubsystem.close()),
                ArmPositionCommand.createCommand(armSubsystem, ArmPosition.OUT),
                new ParallelCommandGroup(
                        new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry).withTimeout(2000),
                        new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, 0, -0.6, -0.0, -0.9, 19)
                ),

                new WaitCommand(200),
                new ScoreCommand(armSubsystem, clawSubsystem),
                new WaitCommand(75),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN)
                        ),
                        new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.INTAKE, telemetry),
                        backUp(10, 0)
                )
        );
//
    }

    public Command lastSampleHighBasket()  {
        return new SequentialCommandGroup(
          new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -90).withTimeout(750),
          forward(9, -90),
          forward(7, -45),
          new TurnToHeadingCommand(mecanumDriveSubsystem, imuSubsystem, telemetry, -45),
//          forward(2, -45),
          new UnInstantCommand(()-> clawSubsystem.close()),
          new WaitCommand(500),
          backUp(5, -45),
          new TurnToHeadingCommand (mecanumDriveSubsystem, imuSubsystem, telemetry, 0),
          new ParallelCommandGroup(
                  ArmPositionCommand.createCommand(armSubsystem, ArmPosition.OUT),
                  new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.UPPERBASKET, telemetry).withTimeout(2000),
                  new EncoderDriveCommand(mecanumDriveSubsystem, imuSubsystem, 0, -.85, 0, -0.85, 20)
          ),
          new WaitCommand(200),
          new ScoreCommand(armSubsystem, clawSubsystem),
          new WaitCommand(75),
          new ParallelCommandGroup(
                  new ElevatorPosCommand(elevatorSubsystem, ElevatorPosition.INTAKE, telemetry).withTimeout(2000),
                  backUp(10, 0),
                  new SequentialCommandGroup(
                       new WaitCommand(300),
                       ArmPositionCommand.createCommand(armSubsystem, ArmPosition.IN)
                  )
          )

        );
    }
}

