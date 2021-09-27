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
import org.firstinspires.ftc.teamcode.MotorGroupTemp;

public class DriveSubsystem extends SubsystemBase {

    private MotorGroupTemp leftDrive, rightDrive;
    private Motor.Encoder leftEncoders, rightEncoders;
    private DifferentialDrive differentialArcade;
    private DifferentialDriveOdometry driveOdometry;
    private RevIMU imu;

    public DriveSubsystem(MotorGroupTemp left, MotorGroupTemp right, RevIMU revIMU) {
        leftDrive = left;
        rightDrive = right;
        leftEncoders = leftDrive.encoder;
        rightEncoders = rightDrive.encoder;
        leftEncoders.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        rightEncoders.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        imu = revIMU;

        resetEncoders();
        differentialArcade = new DifferentialDrive(leftDrive, rightDrive);
        driveOdometry = new DifferentialDriveOdometry(imu.getRotation2d());
    }

    public void resetEncoders() {
        leftEncoders.reset();
        rightEncoders.reset();
    }

    public Pose2d getPose() {
        return driveOdometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftDrive.getPositions().get(0), rightDrive.getPositions().get(0));
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
        driveOdometry.update(imu.getRotation2d(), leftEncoders.getDistance(),
                rightEncoders.getDistance());
    }
}
