package org.firstinspires.ftc.teamcode.rr.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.rr.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.rr.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;

/**
 * This is a simple routine to test turning capabilities.
 * <p>
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Config
@Autonomous(group = "drive")
public class RRTurnTest extends CommandOpMode {

    public static double ANGLE = 90; // deg

    private TankDriveSubsystem drive;
    private TurnCommand turnCommand;

    @Override
    public void initialize() {
        drive = new TankDriveSubsystem(new SampleTankDrive(hardwareMap));
        turnCommand = new TurnCommand(drive, Math.toRadians(ANGLE));
        schedule(turnCommand.whenFinished(() -> {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        }));
    }

}