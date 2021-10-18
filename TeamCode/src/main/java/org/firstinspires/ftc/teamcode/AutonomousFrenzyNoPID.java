package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "robotTwo")
public class AutonomousFrenzyNoPID extends CommandOpMode {

    private Motor frontLeft, backLeft, frontRight, backRight;
    private MotorGroupTemp leftDrive, rightDrive;

    private RevIMU imu;

    private DifferentialDriveKinematics driveKinematics;
    private DriveSubsystem driveSubsystem;
    private RamseteCommand ramseteCommand;

    @Override
    public void initialize() {
        frontLeft = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_435);
        backLeft = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_435);
        frontRight = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_435);
        backRight = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_435);

        leftDrive = new MotorGroupTemp(frontLeft, backLeft);
        rightDrive = new MotorGroupTemp(frontRight, backRight);
        rightDrive.setInverted(true);

        frontLeft.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        backRight.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        backLeft.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        frontRight.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        backRight.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);


        imu = new RevIMU(hardwareMap);
        imu.init();
        driveKinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);

        driveSubsystem = new DriveSubsystem(leftDrive, rightDrive, imu);
        ramseteCommand = new RamseteCommand(TestTrajectory.generateTrajectory(), driveSubsystem::getPose,
                new RamseteController(DriveConstants.B, DriveConstants.ZETA),
                driveKinematics,
                driveSubsystem::driveAuton);

        schedule(new ParallelCommandGroup(
                ramseteCommand,
                new RunCommand(() -> {
//                    telemetry.addData("CurPos", driveSubsystem.getPose());
//                    telemetry.addData("Left Encoders", leftDrive.getPositions());
//                    telemetry.addData("Right Encoders", rightDrive.getPositions());
                    telemetry.addData("Left Speeds", leftDrive.getSpeeds());
                    telemetry.addData("Right Speeds", rightDrive.getSpeeds());
                    telemetry.addData("Wheel Speeds", driveSubsystem.getWheelSpeeds());
                    telemetry.addData("fl", frontLeft.getCorrectedVelocity());
                    telemetry.addData("fr", frontRight.getCorrectedVelocity());
                    telemetry.addData("bl", backLeft.getCorrectedVelocity());
                    telemetry.addData("br", backRight.getCorrectedVelocity());
                    telemetry.update();
                })
        ));
    }

}