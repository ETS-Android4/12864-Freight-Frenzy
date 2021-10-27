package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;

public class TestTrajectory {
    public static Trajectory generateTrajectory() {

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

    public static Trajectory Rohith() {
        Pose2d start = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        Pose2d end = new Pose2d(3, 0.0, new Rotation2d(0.0));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();

        interiorWaypoints.add(new Translation2d(0.95, 0.0));
        interiorWaypoints.add(new Translation2d(1.2, 0.6));
        interiorWaypoints.add(new Translation2d(1.6, 0.6));
        interiorWaypoints.add(new Translation2d(2.2, 0.0));

        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.5);
        config.setReversed(true);

        return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    }


    public static Trajectory Min() {
        Pose2d start = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        Pose2d end = new Pose2d(3, 0.0, new Rotation2d(0.0));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();

        interiorWaypoints.add(new Translation2d(1.1, 0.0));
        interiorWaypoints.add(new Translation2d(1.3, 0.5));
        interiorWaypoints.add(new Translation2d(1.6, 0.6));
        interiorWaypoints.add(new Translation2d(2.4, 0.0));

        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.5);
        config.setReversed(true);

        return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    }
}
