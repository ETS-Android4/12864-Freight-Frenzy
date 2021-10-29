package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MotorGroupTemp;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.DriveConstants.kS;
import static org.firstinspires.ftc.teamcode.DriveConstants.kV;

@Config
@Autonomous(group = "Tuning")
public class ManualFeedforwardTuner extends CommandOpMode {

    public static double DISTANCE = 2;
    private boolean movingForwards;
    private double profileStart, profileTime, prevVelo, prevTime;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private Motor frontLeft, backLeft, frontRight, backRight;
    private MotorGroupTemp leftDrive, rightDrive;

    private RevIMU imu;
    private ElapsedTime clock;

    private GamepadEx gamepad;
    private Button xButton, aButton;

    private DriveSubsystem driveSubsystem;

    private TrapezoidProfile motionProfile;
    private SimpleMotorFeedforward feedForward;

    private Mode mode;

    private static TrapezoidProfile genMotionProfile(boolean movingForward) {
        TrapezoidProfile.State setPoint = new TrapezoidProfile.State((movingForward ? 0 : DISTANCE), 0);
        TrapezoidProfile.State goal = new TrapezoidProfile.State((movingForward ? DISTANCE : 0), 0);
        return new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL), goal, setPoint);
    }

    @Override
    public void initialize() {
        frontLeft = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_435);
        backLeft = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_435);
        frontRight = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_435);
        backRight = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_435);

        leftDrive = new MotorGroupTemp(frontLeft, backLeft);
        rightDrive = new MotorGroupTemp(frontRight, backRight);
        rightDrive.setInverted(true);

        gamepad = new GamepadEx(gamepad1);

        imu = new RevIMU(hardwareMap);
        imu.init();

        mode = Mode.TUNING_MODE;

        feedForward = new SimpleMotorFeedforward(kS, kV, kA);

        clock = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveSubsystem = new DriveSubsystem(leftDrive, rightDrive, imu, telemetry);

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        profileTime = 0;
        prevVelo = 0;

        schedule(new InstantCommand(() -> {
            movingForwards = true;
            motionProfile = genMotionProfile(true);
            profileStart = clock.seconds();
        }), new RunCommand(() -> telemetry.addData("mode", mode)));

        xButton = new GamepadButton(gamepad, GamepadKeys.Button.X)
                .whenPressed(() -> mode = Mode.DRIVER_MODE);
        aButton = new GamepadButton(gamepad, GamepadKeys.Button.A)
                .whenPressed(() -> {
                    mode = Mode.TUNING_MODE;
                    movingForwards = true;
                    motionProfile = genMotionProfile(true);
                    profileStart = clock.seconds();
                });

        schedule(new RunCommand(() -> {
            switch (mode) {
                case TUNING_MODE:
                    // calculate and set the motor power
                    prevTime = profileTime;
                    profileTime = clock.seconds() - profileStart;

                    if (profileTime > motionProfile.totalTime()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        motionProfile = genMotionProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    TrapezoidProfile.State motionState = motionProfile.calculate(profileTime);
                    double targetPower = feedForward.calculate(motionState.velocity,
                            (motionState.velocity - prevVelo) / (profileTime - prevTime));
                    prevVelo = motionState.velocity;

                    driveSubsystem.driveAuton(targetPower, targetPower);

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.velocity);
                    telemetry.addData("measuredVelocity", driveSubsystem.getWheelSpeeds().leftMetersPerSecond);
                    telemetry.addData("error", motionState.velocity - driveSubsystem.getWheelSpeeds().leftMetersPerSecond);
                    break;
                case DRIVER_MODE:
                    driveSubsystem.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
                    break;
            }
        }).alongWith(new RunCommand(telemetry::update)));
    }

}
