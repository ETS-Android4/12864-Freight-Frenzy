package org.firstinspires.ftc.teamcode.commands.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.rr.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropOffSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinnerSubsystem;

public class DuckParkRed extends SequentialCommandGroup {

    private Pose2d startPos = new Pose2d(-36.0, -63.0, Math.toRadians(90.0));

    public DuckParkRed(TankDriveSubsystem driveSubsystem,
                       ElapsedTime time, DropOffSubsystem dropOffSubsystem, int location,
                       SpinnerSubsystem spinnerSubsystem) {

        driveSubsystem.setPoseEstimate(startPos);

        Trajectory traj0 = driveSubsystem.trajectoryBuilder(startPos)
                .forward(5.0)
                .build();

        Trajectory traj1 = driveSubsystem.trajectoryBuilder(
                new Pose2d(traj0.end().vec(), traj0.end().getHeading() + Math.toRadians(-90.0)))
                .forward(25.0)
                .build();

        Trajectory traj2 = driveSubsystem.trajectoryBuilder(traj1.end())
                .back(10.0)
                .build();

        Trajectory traj3 = driveSubsystem.trajectoryBuilder(traj2.end())
                .forward(20.5)
                .build();

        Trajectory traj4 = driveSubsystem.trajectoryBuilder(traj3.end())
                .back(10.0)
                .build();

        addCommands(
                new TrajectoryFollowerCommand(driveSubsystem, traj0),
                new TurnCommand(driveSubsystem, Math.toRadians(90.0)).
                        alongWith(new InstantCommand(() -> spinnerSubsystem.spin(0.35))
                        ),
                new TrajectoryFollowerCommand(driveSubsystem, traj1),
                new WaitCommand(3500),
                new TrajectoryFollowerCommand(driveSubsystem, traj2),
                new TurnCommand(driveSubsystem, Math.toRadians(-95.0)).
                        alongWith(new InstantCommand(() -> spinnerSubsystem.stop())
                        ),
                new TrajectoryFollowerCommand(driveSubsystem, traj3),
                new TurnCommand(driveSubsystem, Math.toRadians(-95.0)),
                new TrajectoryFollowerCommand(driveSubsystem, traj4)
        );

    }

}
