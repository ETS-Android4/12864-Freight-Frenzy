package org.firstinspires.ftc.teamcode.soMuchAuto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.paths.DuckSideRed;
import org.firstinspires.ftc.teamcode.commands.paths.FreightSideRed;
import org.firstinspires.ftc.teamcode.rr.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropOffSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.SpinnerSubsystem;
import org.firstinspires.ftc.teamcode.vision.CapstoneDetector;

import java.util.HashMap;

@Autonomous(name = "Red Duck")
public class RedDuckAuto extends CommandOpMode {

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
        capstoneDetector.init();

        tankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(tankDrive);

        liftSubsystemNoPID = new LiftSubsystemNoPID(liftMotor);

        dropOffSubsystem = new DropOffSubsystem(dropOffLeft, dropOffRight);

        spinnerSubsystem = new SpinnerSubsystem(duckSpinner);

        schedule(new WaitUntilCommand(this::isStarted).andThen(new RunCommand(() -> {
            telemetry.addData("location", location);
            telemetry.update();
        })));

        schedule(new WaitUntilCommand(this::isStarted).andThen(new WaitCommand(500))
                .andThen(new RunCommand(() -> location = capstoneDetector.getPlacement()).
                        raceWith(new WaitCommand(500))).andThen(
                        new SelectCommand(new HashMap<Object, Command>() {{
                            put(CapstoneDetector.Placement.RIGHT,
                                    (new DuckSideRed(tankDriveSubsystem, liftSubsystemNoPID,
                                            time, dropOffSubsystem, 1, spinnerSubsystem)));
                            put(CapstoneDetector.Placement.CENTER,
                                    (new DuckSideRed(tankDriveSubsystem, liftSubsystemNoPID,
                                            time, dropOffSubsystem, 0, spinnerSubsystem)));
                            put(CapstoneDetector.Placement.LEFT,
                                    (new DuckSideRed(tankDriveSubsystem, liftSubsystemNoPID,
                                            time, dropOffSubsystem, 3, spinnerSubsystem)));
                        }}, () -> location
                        )));
    }
}