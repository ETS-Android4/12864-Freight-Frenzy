package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

public class DriveSubsystem extends SubsystemBase {

    private MotorGroup leftDrive, rightDrive;
    private DifferentialDrive differentialArcade;

    public DriveSubsystem(MotorGroup left, MotorGroup right) {
        leftDrive = left;
        rightDrive = right;

        differentialArcade = new DifferentialDrive(leftDrive, rightDrive);
    }

    public void drive(double forwardSpeed, double turnSpeed) {
        differentialArcade.arcadeDrive(forwardSpeed, turnSpeed);
    }
}
