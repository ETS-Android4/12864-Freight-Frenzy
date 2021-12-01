package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp()
public class LocalizationTest extends CommandOpMode {

    private Motor frontLeft, backLeft, frontRight, backRight;
    private MotorGroupTemp leftDrive, rightDrive;

    private GamepadEx gamepad;
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private RevIMU imu;

    @Override
    public void initialize() {

        frontLeft = new Motor(hardwareMap, "fL");
        backLeft = new Motor(hardwareMap, "bL");
        frontRight = new Motor(hardwareMap, "fR");
        backRight = new Motor(hardwareMap, "bR");

        leftDrive = new MotorGroupTemp(frontLeft, backLeft);
        rightDrive = new MotorGroupTemp(frontRight, backRight);

        rightDrive.setInverted(true);

        gamepad = new GamepadEx(gamepad1);
        imu = new RevIMU(hardwareMap);
        imu.init();


        driveSubsystem = new DriveSubsystem(leftDrive, rightDrive, imu, telemetry);

        schedule(new RunCommand(() -> {
            Pose2d poseEstimate = driveSubsystem.getPose();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }));

        driveCommand = new DriveCommand(driveSubsystem, gamepad::getRightX, gamepad::getLeftY);

        schedule(driveCommand);
    }
}
