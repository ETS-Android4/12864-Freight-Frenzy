package org.firstinspires.ftc.teamcode.commands.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.rr.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;

public class FreightSideRed extends SequentialCommandGroup {

    private Pose2d startPos = new Pose2d(12.0, -63.0, Math.toRadians(90.0));

    public FreightSideRed(TankDriveSubsystem driveSubsystem) {
        driveSubsystem.setPoseEstimate(startPos);

        Trajectory traj0 = driveSubsystem.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-12.0, -60.0), Math.toRadians(270.0))
                .build();

        Trajectory traj1 = driveSubsystem.trajectoryBuilder(traj0.end())
                .forward(-16.3)
                .build();

        Trajectory traj2 = driveSubsystem.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(55.0, -47.0), 0.0)
                .build();

        addCommands(
                new TrajectoryFollowerCommand(driveSubsystem, traj0),
                new TrajectoryFollowerCommand(driveSubsystem, traj1),
                new WaitCommand(2000),
                new TrajectoryFollowerCommand(driveSubsystem, traj2)
        );
    }
}
