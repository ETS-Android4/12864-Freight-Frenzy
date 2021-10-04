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
    private DifferentialDrive differentialArcade;
    private DifferentialDriveOdometry driveOdometry;
    private RevIMU imu;

    public DriveSubsystem(MotorGroupTemp left, MotorGroupTemp right, RevIMU revIMU) {
        leftDrive = left;
        rightDrive = right;
        leftDrive.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        rightDrive.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        imu = revIMU;

        resetEncoders();
        differentialArcade = new DifferentialDrive(leftDrive, rightDrive);
        driveOdometry = new DifferentialDriveOdometry(imu.getRotation2d());
    }

    public void resetEncoders() {
        leftDrive.resetEncoder();
        rightDrive.resetEncoder();
    }

    public Pose2d getPose() {
        return driveOdometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds
                (leftDrive.getVelocity()*DriveConstants.DISTANCE_PER_PULSE,
                        rightDrive.getVelocity()*DriveConstants.DISTANCE_PER_PULSE);
    }

    public void driveAuton(double leftSpeed, double rightSpeed){
        leftDrive.set(leftSpeed);
        rightDrive.set(rightSpeed);
    }

    public void drive(double forwardSpeed, double turnSpeed) {
        differentialArcade.arcadeDrive(-forwardSpeed, -turnSpeed, true);
    }

    @Override
    public void periodic() {
        driveOdometry.update(imu.getRotation2d(), leftDrive.getPositions().get(0),
                rightDrive.getPositions().get(0));
    }
}
