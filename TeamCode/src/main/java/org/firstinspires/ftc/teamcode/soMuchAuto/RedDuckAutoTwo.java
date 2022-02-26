package org.firstinspires.ftc.teamcode.soMuchAuto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.paths.DuckParkBlue;
import org.firstinspires.ftc.teamcode.commands.paths.DuckParkRed;
import org.firstinspires.ftc.teamcode.rr.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropOffSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.SpinnerSubsystem;
import org.firstinspires.ftc.teamcode.vision.CapstoneDetector;

import java.util.HashMap;

@Autonomous(name = "RedRedRed")
public class RedDuckAutoTwo extends CommandOpMode {

    private Motor liftMotor;
    private Motor duckSpinner;

    private SimpleServo dropOffLeft;
    private SimpleServo dropOffRight;

    private ElapsedTime time;

    private CapstoneDetector capstoneDetector;

    private SampleTankDrive tankDrive;
    private TankDriveSubsystem tankDriveSubsystem;

    private LiftSubsystemNoPID liftSubsystemNoPID;
    private DropOffSubsystem dropOffSubsystem;
    private SpinnerSubsystem spinnerSubsystem;

    private CapstoneDetector.Placement location;

    @Override
    public void initialize() {

        liftMotor = new Motor(hardwareMap, "liftM", Motor.GoBILDA.RPM_312);

        duckSpinner = new Motor(hardwareMap, "dS");

        dropOffLeft = new SimpleServo(hardwareMap, "dropSL", -90, 90);
        dropOffRight = new SimpleServo(hardwareMap, "dropSR", -90, 90);

        time = new ElapsedTime();

        capstoneDetector = new CapstoneDetector(hardwareMap, "coolio");

        tankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(tankDrive);

        liftSubsystemNoPID = new LiftSubsystemNoPID(liftMotor);

        dropOffSubsystem = new DropOffSubsystem(dropOffLeft, dropOffRight);

        spinnerSubsystem = new SpinnerSubsystem(duckSpinner);

        schedule(new WaitUntilCommand(this::isStarted)
                .andThen(new InstantCommand(() -> capstoneDetector.init()
                )).andThen(new WaitCommand(2000))
                .andThen(new RunCommand(() -> {
                    location = capstoneDetector.getPlacement();
                    telemetry.addData("Placement ", location);
                    telemetry.update();
                })
                        .raceWith(new WaitCommand(1000))).andThen(
                        new SelectCommand(new HashMap<Object, Command>() {{
                            put(CapstoneDetector.Placement.RIGHT,
                                    (new InstantCommand(() -> capstoneDetector.getCamera().stopStreaming())
                                            .andThen(new DuckParkRed(tankDriveSubsystem,
                                                    time, dropOffSubsystem, 1, spinnerSubsystem))));
                            put(CapstoneDetector.Placement.CENTER,
                                    (new InstantCommand(() -> capstoneDetector.getCamera().stopStreaming())
                                            .andThen(new DuckParkRed(tankDriveSubsystem,
                                                    time, dropOffSubsystem, 0, spinnerSubsystem))));
                            put(CapstoneDetector.Placement.LEFT,
                                    (new InstantCommand(() -> capstoneDetector.getCamera().stopStreaming())
                                            .andThen(new DuckParkRed(tankDriveSubsystem,
                                                    time, dropOffSubsystem, 3, spinnerSubsystem))));
                        }}, () -> location
                        )));
    }
}