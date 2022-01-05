package org.firstinspires.ftc.teamcode.commands.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
//import com.amongus.sus.imposterpngRED;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DropOffCommand;
import org.firstinspires.ftc.teamcode.rr.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.rr.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropOffSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

public class FreightSideRed extends SequentialCommandGroup {

    private Pose2d startPos = new Pose2d(12.0, -63.0, Math.toRadians(90.0));

    public FreightSideRed(TankDriveSubsystem driveSubsystem, LiftSubsystemNoPID liftSubsystem,
                          ElapsedTime time, DropOffSubsystem dropOffSubsystem) {
        driveSubsystem.setPoseEstimate(startPos);

        Trajectory traj0 = driveSubsystem.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-12.0, -60.0), Math.toRadians(270.0))
                .build();

        Trajectory traj1 = driveSubsystem.trajectoryBuilder(traj0.end())
                .back(18)
                .build();

        Trajectory traj2 = driveSubsystem.trajectoryBuilder(traj1.end())
                .forward(5.8)
                .build();

        Trajectory traj3 = driveSubsystem.trajectoryBuilder(
                new Pose2d(traj2.end().vec(), (traj2.end().getHeading() + Math.toRadians(95.0))))
                .splineTo(new Vector2d(60.0, -40.0), 0.0)
                .build();

        addCommands(
                new TrajectoryFollowerCommand(driveSubsystem, traj0),
                new TurnCommand(driveSubsystem, Math.toRadians(-10.0)),
                new TrajectoryFollowerCommand(driveSubsystem, traj1),
                new InstantCommand(() -> dropOffSubsystem.drop()),
                new WaitCommand(2000),
                new InstantCommand(() -> dropOffSubsystem.returnHome()),
                new WaitCommand(1000),
                new TrajectoryFollowerCommand(driveSubsystem, traj2),
                new TurnCommand(driveSubsystem, Math.toRadians(95.0)),
                new TrajectoryFollowerCommand(driveSubsystem, traj3)
        );
    }
}
