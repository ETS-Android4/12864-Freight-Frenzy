package org.firstinspires.ftc.teamcode.soMuchAuto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.paths.FreightSideRed;
import org.firstinspires.ftc.teamcode.rr.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropOffSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

@Autonomous(name = "Poggers")
public class RRAutoTesting extends CommandOpMode {

    private Motor liftMotor;

    private SimpleServo dropOffLeft;
    private SimpleServo dropOffRight;

    private ElapsedTime time;

    private SampleTankDrive tankDrive;
    private TankDriveSubsystem tankDriveSubsystem;

    private LiftSubsystemNoPID liftSubsystemNoPID;

    private DropOffSubsystem dropOffSubsystem;


    @Override
    public void initialize() {

        liftMotor = new Motor(hardwareMap, "liftM", Motor.GoBILDA.RPM_312);

        dropOffLeft = new SimpleServo(hardwareMap, "dropSL", -90, 90);
        dropOffRight = new SimpleServo(hardwareMap, "dropSR", -90, 90);

        time = new ElapsedTime();

        tankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(tankDrive);

        liftSubsystemNoPID = new LiftSubsystemNoPID(liftMotor);

        dropOffSubsystem = new DropOffSubsystem(dropOffLeft, dropOffRight);

        schedule(new WaitUntilCommand(this::isStarted).andThen(
                new FreightSideRed(tankDriveSubsystem, liftSubsystemNoPID, time, dropOffSubsystem)));

    }
}
