package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {

    //TODO: Just tune all of this it all needs to be changed
    public static double TRACK_WIDTH = 0.38227;
    public static double WHEEL_DIAMETER = 0.1;
    public static double TICKS_PER_REV = 383.6;
    public static double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    // Values for ramsete controller
    public static double B = 1.0;
    public static double ZETA = 0.0;

    // Feedforward values for drive
    public static double kV = 1.0;
    public static double kA = 0.0;
    public static double kS = 0.0;

    // Drive PID
    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

}
