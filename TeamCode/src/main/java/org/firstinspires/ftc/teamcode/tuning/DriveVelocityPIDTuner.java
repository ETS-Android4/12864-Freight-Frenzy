package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.MotorGroupTemp;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.DriveConstants.*;

@Config
@Autonomous(name = "TuneVeloUwU")
public class DriveVelocityPIDTuner extends CommandOpMode {

    public static double DISTANCE = 2;
    private boolean movingForwards;
    private double lastKp, lastKi, lastKd, profileStart;

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

        lastKp = kP;
        lastKi = kI;
        lastKd = kD;

        clock = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveSubsystem = new DriveSubsystem(leftDrive, rightDrive, imu, telemetry);

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        schedule(new SequentialCommandGroup(
                new WaitUntilCommand(this::isStarted),
                new InstantCommand(() -> {
                    movingForwards = true;
                    motionProfile = genMotionProfile(true);
                    profileStart = clock.seconds();
                })
        ), new RunCommand(() -> telemetry.addData("mode", mode)));

        xButton = new GamepadButton(gamepad, GamepadKeys.Button.X)
                .whenPressed(() -> {
                    mode = Mode.DRIVER_MODE;
//                    drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                });
        aButton = new GamepadButton(gamepad, GamepadKeys.Button.A)
                .whenPressed(() -> {
//                    drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    mode = Mode.TUNING_MODE;
                    movingForwards = true;
                    motionProfile = genMotionProfile(true);
                    profileStart = clock.seconds();
                });

        schedule(new RunCommand(() -> {
            switch (mode) {
                case TUNING_MODE:
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > motionProfile.totalTime()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        motionProfile = genMotionProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    TrapezoidProfile.State motionState = motionProfile.calculate(profileTime);
                    double targetPower = kV * motionState.velocity;
                    driveSubsystem.driveAuton(targetPower, targetPower);

                    DifferentialDriveWheelSpeeds velocities = driveSubsystem.getWheelSpeeds();

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.velocity);
                    telemetry.addData("measuredVelocityLeft", velocities.leftMetersPerSecond);
                    telemetry.addData("measuredVelocityRight", velocities.rightMetersPerSecond);
                    telemetry.addData(
                            "errorLeft",
                            motionState.velocity - velocities.leftMetersPerSecond
                    );
                    telemetry.addData(
                            "errorRight",
                            motionState.velocity - velocities.rightMetersPerSecond
                    );
                    break;
                case DRIVER_MODE:
                    driveSubsystem.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
                    break;
            }

            if (lastKp != kP || lastKd != kD
                    || lastKi != kI) {

                lastKp = kP;
                lastKi = kI;
                lastKd = kD;
            }

            telemetry.update();
        }));
    }

}
