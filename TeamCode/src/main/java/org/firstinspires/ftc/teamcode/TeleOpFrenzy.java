package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommandNoPID;
import org.firstinspires.ftc.teamcode.commands.SpinnerCommand;
import org.firstinspires.ftc.teamcode.commands.StartEndCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.DropOffCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropOffSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.SpinnerSubsystem;

@TeleOp(name = "MainTeleop")
public class TeleOpFrenzy extends CommandOpMode {

    private Motor frontLeft, backLeft, frontRight, backRight;
    private MotorGroupTemp leftDrive, rightDrive;
    private Motor duckSpinner;
    private Motor liftMotor;
    private Motor intakeMotor;

    private GamepadEx driver;
    private RevIMU imu;
    private ElapsedTime time;

    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private SpinnerSubsystem spinnerSubsystem;
    private SpinnerCommand spinnerCommand, spinnerCommandTwo;

    private LiftSubsystem liftSubsystem;
    private LiftCommand liftCommand;
    private LiftSubsystemNoPID liftSubsystemNoPID;
    private LiftCommandNoPID liftCommandNoPID;

    private SimpleServo dropOffLeft;
    private SimpleServo dropOffRight;

    private DropOffSubsystem dropOffSubsystem;
    private DropOffCommand dropOffCommand;

    private double multiplier = 1.0;

    @Override
    public void initialize() {
        frontLeft = new Motor(hardwareMap, "fL");
        backLeft = new Motor(hardwareMap, "bL");
        frontRight = new Motor(hardwareMap, "fR");
        backRight = new Motor(hardwareMap, "bR");

        leftDrive = new MotorGroupTemp(frontLeft, backLeft);
        rightDrive = new MotorGroupTemp(frontRight, backRight);
        rightDrive.setInverted(true);

        duckSpinner = new Motor(hardwareMap, "dS");

        liftMotor = new Motor(hardwareMap, "liftM", Motor.GoBILDA.RPM_312);

        intakeMotor = new Motor(hardwareMap, "intake");

        dropOffLeft = new SimpleServo(hardwareMap, "dropSL", -90, 90);
        dropOffRight = new SimpleServo(hardwareMap, "dropSR", -90, 90);

        driver = new GamepadEx(gamepad1);
        imu = new RevIMU(hardwareMap);
        imu.init();
        time = new ElapsedTime();

        driveSubsystem = new DriveSubsystem(leftDrive, rightDrive, imu, telemetry);
        driveCommand = new DriveCommand(driveSubsystem, driver::getLeftY, driver::getRightX, () -> multiplier);

        spinnerSubsystem = new SpinnerSubsystem(duckSpinner);
        spinnerCommand = new SpinnerCommand(spinnerSubsystem, true);
        spinnerCommandTwo = new SpinnerCommand(spinnerSubsystem, false);

        liftSubsystem = new LiftSubsystem(liftMotor);
        liftCommand = new LiftCommand(liftSubsystem, telemetry);
        liftSubsystemNoPID = new LiftSubsystemNoPID(liftMotor);
        liftCommandNoPID = new LiftCommandNoPID(liftSubsystemNoPID, time);


        dropOffSubsystem = new DropOffSubsystem(dropOffLeft, dropOffRight);
        dropOffCommand = new DropOffCommand(dropOffSubsystem);

        // Shreya had never listened to Drake until like a year after she move to Toronto
        // TODO: Change to StartEndCommand
        driver.getGamepadButton(GamepadKeys.Button.A).whenHeld(spinnerCommand);
        driver.getGamepadButton(GamepadKeys.Button.B).whenHeld(spinnerCommandTwo);
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(liftCommand);
        driver.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(dropOffCommand);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(liftCommandNoPID);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new StartEndCommand(() -> intakeMotor.set(0.9), () -> intakeMotor.stopMotor()));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new StartEndCommand(() -> intakeMotor.set(-0.9), () -> intakeMotor.stopMotor()));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenHeld(new StartEndCommand(() -> liftSubsystemNoPID.slowMotorDown(), () -> liftSubsystemNoPID.motorStop(), liftSubsystemNoPID));
        driver.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(new InstantCommand(() -> multiplier = 0.65), new InstantCommand(() -> multiplier = 1.0));

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
