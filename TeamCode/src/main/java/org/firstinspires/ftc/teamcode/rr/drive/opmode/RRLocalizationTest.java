package org.firstinspires.ftc.teamcode.rr.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.rr.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;

@TeleOp(group = "drive")
public class RRLocalizationTest extends CommandOpMode {

    private TankDriveSubsystem drive;
    private DriveCommand driveCommand;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        drive = new TankDriveSubsystem(new SampleTankDrive(hardwareMap));

        gamepad = new GamepadEx(gamepad1);

        schedule(new RunCommand(() -> {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }));

        driveCommand = new DriveCommand(
                drive, () -> gamepad.getLeftY(), gamepad::getRightX
        );

        schedule(driveCommand);
    }

}
