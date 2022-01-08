package org.firstinspires.ftc.teamcode.commands.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.LiftCommandNoPID;
import org.firstinspires.ftc.teamcode.commands.SpinnerCommand;
import org.firstinspires.ftc.teamcode.rr.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.rr.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.rr.drive.RRDriveConstants;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropOffSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;
import org.firstinspires.ftc.teamcode.subsystems.SpinnerSubsystem;

import java.util.Arrays;

public class DuckSideRed extends SequentialCommandGroup {

    private Pose2d startPos = new Pose2d(-36.0, -63.0, Math.toRadians(90.0));

    public DuckSideRed(TankDriveSubsystem driveSubsystem, LiftSubsystemNoPID liftSubsystem,
                       ElapsedTime time, DropOffSubsystem dropOffSubsystem, int location,
                       SpinnerSubsystem spinnerSubsystem) {

        driveSubsystem.setPoseEstimate(startPos);

        Trajectory traj0 = driveSubsystem.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-12.0, -50.0), Math.toRadians(270.0))
                .build();

        Trajectory traj1 = driveSubsystem.trajectoryBuilder(new Pose2d(traj0.end().vec(),
                traj0.end().getHeading() + Math.toRadians(17.0)))
                .back(11.3)
                .build();

        Trajectory traj2 = driveSubsystem.trajectoryBuilder(traj1.end())
                .forward(5.5)
                .build();

        Trajectory traj3 = driveSubsystem.trajectoryBuilder(traj2.end())
                .forward(36.0)
                .build();

        Trajectory traj4 = driveSubsystem.trajectoryBuilder(traj3.end())
                .forward(20.0, new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(RRDriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(40, RRDriveConstants.TRACK_WIDTH)
                        )),
                        new ProfileAccelerationConstraint(35.0))
                .build();

        Trajectory traj5 = driveSubsystem.trajectoryBuilder(traj4.end())
                .back(25.0)
                .build();

        Trajectory traj6 = driveSubsystem.trajectoryBuilder(traj5.end())
                .forward(15)
                .build();

        addCommands(
                new TrajectoryFollowerCommand(driveSubsystem, traj0),
                new TurnCommand(driveSubsystem, Math.toRadians(17.0)).
                        alongWith(new LiftCommandNoPID(liftSubsystem, time, location)
                        ),
                new TrajectoryFollowerCommand(driveSubsystem, traj1),
                new WaitCommand(1000),
                new InstantCommand(() -> dropOffSubsystem.drop()),
                new WaitCommand(2000),
                new InstantCommand(() -> dropOffSubsystem.returnHome()),
                new WaitCommand(500),
                new LiftCommandNoPID(liftSubsystem, time, 4),
                new WaitCommand(1000),
                new TrajectoryFollowerCommand(driveSubsystem, traj2)
                        .alongWith(new LiftCommandNoPID(liftSubsystem, time, 2)),
                new TurnCommand(driveSubsystem, Math.toRadians(-80.0)),
                new TrajectoryFollowerCommand(driveSubsystem, traj3),
                new TurnCommand(driveSubsystem, Math.toRadians(65.0)),
                new TrajectoryFollowerCommand(driveSubsystem, traj4)
                        .alongWith(new InstantCommand(() -> spinnerSubsystem.spin(0.35))
                        ),
                new WaitCommand(2000),
                new InstantCommand(() -> spinnerSubsystem.stop()),
                new TrajectoryFollowerCommand(driveSubsystem, traj5),
                new TurnCommand(driveSubsystem, Math.toRadians(-75.0)),
                new TrajectoryFollowerCommand(driveSubsystem, traj6)
        );

    }

}
