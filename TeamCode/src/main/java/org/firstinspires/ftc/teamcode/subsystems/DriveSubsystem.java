package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private MotorGroup leftDrive, rightDrive;
    private DifferentialDrive differentialArcade;
    private DifferentialDriveOdometry driveOdometry;
    private RevIMU imu;

    public DriveSubsystem(MotorGroup left, MotorGroup right, RevIMU revIMU) {
        leftDrive = left;
        rightDrive = right;
        leftDrive.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        rightDrive.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        imu = revIMU;

        resetEncoders();
        differentialArcade = new DifferentialDrive(leftDrive, rightDrive);
        driveOdometry = new DifferentialDriveOdometry(imu.getRotation2d());
    }

    public void resetEncoders() {
        leftDrive.encoder.reset();
        rightDrive.encoder.reset();
    }

    public Pose2d getPose() {
        return driveOdometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftDrive.encoder.getDistance(), rightDrive.encoder.getDistance());
    }

    public void driveAuton(double leftSpeed, double rightSpeed){
        leftDrive.set(leftSpeed);
        rightDrive.set(rightSpeed);
    }

    public void drive(double forwardSpeed, double turnSpeed) {
        differentialArcade.arcadeDrive(forwardSpeed, turnSpeed);
    }

    @Override
    public void periodic() {
        driveOdometry.update(imu.getRotation2d(), leftDrive.encoder.getDistance(),
                rightDrive.encoder.getDistance());
    }
}
