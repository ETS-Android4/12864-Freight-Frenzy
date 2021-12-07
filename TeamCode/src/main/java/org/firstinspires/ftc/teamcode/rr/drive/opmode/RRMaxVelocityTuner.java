package org.firstinspires.ftc.teamcode.rr.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.drive.RRDriveConstants;
import org.firstinspires.ftc.teamcode.rr.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum velocity your bot can achieve under load. It
 * will also calculate the effective kF value for your velocity PID.
 * <p>
 * Upon pressing start, your bot will run at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of kF may be desired.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Config
@Autonomous(group = "drive")
public class RRMaxVelocityTuner extends CommandOpMode {

    public static double RUNTIME = 2.0;

    private ElapsedTime timer = new ElapsedTime();
    private double maxVelocity = 0.0;

    private VoltageSensor batteryVoltageSensor;

    private TankDriveSubsystem drive;

    @Override
    public void initialize() {
        drive = new TankDriveSubsystem(new SampleTankDrive(hardwareMap));
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Your bot will go at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        schedule(new WaitUntilCommand(this::isStarted)
                .whenFinished(() -> {
                    telemetry.clearAll();
                    telemetry.update();

                    drive.setDrivePower(new Pose2d(1, 0, 0));
                    timer.reset();
                })
        );

        RunCommand runCommand = new RunCommand(() -> {
            drive.update();

            Pose2d poseVelo = Objects.requireNonNull(
                    drive.getPoseVelocity(),
                    "poseVelocity() must not be null. " +
                            "Ensure that the getWheelVelocities() method has been overridden in your localizer."
            );

            maxVelocity = Math.max(poseVelo.vec().norm(), maxVelocity);
        });

        schedule(new WaitUntilCommand(() -> timer.seconds() >= RUNTIME)
                .deadlineWith(runCommand)
                .whenFinished(() -> {
                    drive.setDrivePower(new Pose2d());

                    double effectiveKf = RRDriveConstants.getMotorVelocityF(veloInchesToTicks(maxVelocity));

                    telemetry.addData("Max Velocity", maxVelocity);
                    telemetry.addData("Voltage Compensated kF", effectiveKf * batteryVoltageSensor.getVoltage() / 12);
                    telemetry.update();
                })
        );
    }

    private double veloInchesToTicks(double inchesPerSec) {
        return inchesPerSec / (2 * Math.PI * RRDriveConstants.WHEEL_RADIUS) / RRDriveConstants.GEAR_RATIO * RRDriveConstants.TICKS_PER_REV;
    }

}