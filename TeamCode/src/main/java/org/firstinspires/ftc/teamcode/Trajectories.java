package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;

@Config
public class Trajectories {

    public static double endX = 1.0, endY = 0.8;


    public static Trajectory testTraj() {

        Pose2d sideStart = new Pose2d(0.0, 0.0,
                Rotation2d.fromDegrees(0));
        Pose2d crossScale = new Pose2d(3.0, 0.0,
                Rotation2d.fromDegrees(0));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(1, 0));
        interiorWaypoints.add(new Translation2d(2, -1));

        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.5);
        config.setReversed(true);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
        return trajectory;
    }

    public static Trajectory traj1() {
        Pose2d start = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        Pose2d end = new Pose2d(endX, endY, new Rotation2d(0.0));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();

        interiorWaypoints.add(new Translation2d(0.1, 0.3));
        interiorWaypoints.add(new Translation2d(0.4, 0.6));


        TrajectoryConfig config = new TrajectoryConfig(0.4, 0.4);

        return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    }

    public static Trajectory traj2() {
        Pose2d start = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        Pose2d end = new Pose2d(0.6, -0.6, new Rotation2d(0.0));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();

        interiorWaypoints.add(new Translation2d(0.6, -0.2));

        TrajectoryConfig config = new TrajectoryConfig(0.4, 0.4);

        return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    }

    public static Trajectory traj3() {
        Pose2d start = new Pose2d(.0, 0.0, new Rotation2d(0.0));
        Pose2d end = new Pose2d(0.3, -0.8, new Rotation2d(0.0));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();

        interiorWaypoints.add(new Translation2d(0.2, -1.5));

        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.5);

        return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    }

    //Blue zone with boxes and balls
    public static Trajectory traj4() {
        Pose2d start = new Pose2d(0.3, 0.0, new Rotation2d(0.0));

        Pose2d end = new Pose2d(0.1, 1.3, new Rotation2d(0.0));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();

        interiorWaypoints.add(new Translation2d(0.3, 1.0));
        interiorWaypoints.add(new Translation2d(0.2, 1.5));


        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.5);

        return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    }

    //Red zone with boxes and balls
    public static Trajectory traj5() {
        Pose2d start = new Pose2d(0.3, 0.0, new Rotation2d(0.0));

        Pose2d end = new Pose2d(0.0, -1.3, new Rotation2d(0.0));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();

        interiorWaypoints.add(new Translation2d(0.2, -1.0));
        interiorWaypoints.add(new Translation2d(0.1, -1.6));


        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.5);

        return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    }
}

