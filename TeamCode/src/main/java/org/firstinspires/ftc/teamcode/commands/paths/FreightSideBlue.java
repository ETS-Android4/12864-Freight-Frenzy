package org.firstinspires.ftc.teamcode.commands.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
//import com.amongus.sus.imposterpngRED;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DropOffCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommandNoPID;
import org.firstinspires.ftc.teamcode.rr.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.rr.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.rr.drive.RRDriveConstants;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropOffSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class FreightSideBlue extends SequentialCommandGroup {

    private Pose2d startPos = new Pose2d(12.0, 63.0, Math.toRadians(-90.0));

    public FreightSideBlue(TankDriveSubsystem driveSubsystem, LiftSubsystemNoPID liftSubsystem,
                           ElapsedTime time, DropOffSubsystem dropOffSubsystem, int location) {
        driveSubsystem.setPoseEstimate(startPos);

        Trajectory push1 = driveSubsystem.trajectoryBuilder(startPos)
                .forward(25.0,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(RRDriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(40, RRDriveConstants.TRACK_WIDTH)
                        )),
                        new ProfileAccelerationConstraint(35.0))
                .build();

        Trajectory push2 = driveSubsystem.trajectoryBuilder(push1.end())
                .back(25.0,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(RRDriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(40, RRDriveConstants.TRACK_WIDTH)
                        )),
                        new ProfileAccelerationConstraint(35.0))
                .build();

        Trajectory traj0 = driveSubsystem.trajectoryBuilder(push2.end())
                .splineTo(new Vector2d(-12.0, 60.0), Math.toRadians(-270.0))
                .build();

        Trajectory traj1 = driveSubsystem.trajectoryBuilder(traj0.end())
                .back(17.0)
                .build();

        Trajectory traj2 = driveSubsystem.trajectoryBuilder(traj1.end())
                .forward(5.7)
                .build();

        Trajectory traj2Half = driveSubsystem.trajectoryBuilder(traj2.end())
                .forward(4.0)
                .build();

        Trajectory traj3 = driveSubsystem.trajectoryBuilder(
                new Pose2d(traj2.end().vec(), (traj2.end().getHeading() + Math.toRadians(-95.0))))
                .splineTo(new Vector2d(60.0, 40.0), 0.0)
                .build();

        addCommands(
                new TrajectoryFollowerCommand(driveSubsystem, push1),
                new WaitCommand(500),
                new TrajectoryFollowerCommand(driveSubsystem, push2),
                new TrajectoryFollowerCommand(driveSubsystem, traj0),
                new TurnCommand(driveSubsystem, Math.toRadians(12.0))
                        .alongWith(new LiftCommandNoPID(liftSubsystem, time, location)
                        ),
                new TrajectoryFollowerCommand(driveSubsystem, traj1),
                new WaitCommand(1000),
                new InstantCommand(() -> dropOffSubsystem.drop()),
                new WaitCommand(2000),
                new TrajectoryFollowerCommand(driveSubsystem, traj2Half),
                new InstantCommand(() -> dropOffSubsystem.returnHome()),
                new LiftCommandNoPID(liftSubsystem, time, 4),
                new WaitCommand(1000),
                new TrajectoryFollowerCommand(driveSubsystem, traj2)
                        .alongWith(new LiftCommandNoPID(liftSubsystem, time, 2)
                        ),
                new TurnCommand(driveSubsystem, Math.toRadians(-95.0)),
                new TrajectoryFollowerCommand(driveSubsystem, traj3)
        );
    }
}
