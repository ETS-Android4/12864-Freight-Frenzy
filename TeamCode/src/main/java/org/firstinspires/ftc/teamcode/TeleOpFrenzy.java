package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "Vroooom")
public class TeleOpFrenzy extends CommandOpMode {

    private Motor frontLeft, backLeft, frontRight, backRight;
    private MotorGroup leftDrive, rightDrive;

    private GamepadEx driver;
    private RevIMU imu;

    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    @Override
    public void initialize() {
        frontLeft = new Motor(hardwareMap, "fL");
        backLeft = new Motor(hardwareMap, "bL");
        frontRight = new Motor(hardwareMap, "fR");
        backRight = new Motor(hardwareMap, "bR");

        leftDrive = new MotorGroup(frontLeft, backLeft);
        rightDrive = new MotorGroup(frontRight, backRight);

        driver = new GamepadEx(gamepad1);
        imu = new RevIMU(hardwareMap);
        imu.init();

        driveSubsystem = new DriveSubsystem(leftDrive, rightDrive, imu);
        driveCommand = new DriveCommand(driveSubsystem, driver::getLeftY, driver::getRightX);

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
