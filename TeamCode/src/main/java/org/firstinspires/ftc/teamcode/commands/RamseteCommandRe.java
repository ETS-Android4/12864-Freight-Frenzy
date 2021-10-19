package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommandRe extends CommandBase {
    private final ElapsedTime m_timer;
    private final boolean m_usePID;
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private final BiConsumer<Double, Double> m_output;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;
    private Telemetry m_telemetry;

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * PID control and feedforward are handled internally, and outputs are scaled -12 to 12
     * representing units of volts.
     *
     * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
     * this
     * is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory      The trajectory to follow.
     * @param pose            A function that supplies the robot pose - use one of
     *                        the odometry classes to provide this.
     * @param controller      The RAMSETE controller used to follow the trajectory.
     * @param feedforward     The feedforward to use for the drive.
     * @param kinematics      The kinematics for the robot drivetrain.
     * @param wheelSpeeds     A function that supplies the speeds of the left and
     *                        right sides of the robot drive.
     * @param leftController  The PIDController for the left side of the robot drive.
     * @param rightController The PIDController for the right side of the robot drive.
     * @param outputVolts     A function that consumes the computed left and right
     *                        outputs (in volts) for the robot drive.
     */
    public RamseteCommandRe(Trajectory trajectory,
                            Supplier<Pose2d> pose,
                            RamseteController controller,
                            SimpleMotorFeedforward feedforward,
                            DifferentialDriveKinematics kinematics,
                            Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
                            PIDController leftController,
                            PIDController rightController,
                            BiConsumer<Double, Double> outputVolts,
                            Telemetry telemetry) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_follower = controller;
        m_feedforward = feedforward;
        m_kinematics = kinematics;
        m_speeds = wheelSpeeds;
        m_leftController = leftController;
        m_rightController = rightController;
        m_output = outputVolts;

        m_usePID = true;

        m_timer = new ElapsedTime();
        m_telemetry = telemetry;
    }

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds
     * from the RAMSETE controller, and will need to be converted into a usable form by the user.
     *
     * @param trajectory            The trajectory to follow.
     * @param pose                  A function that supplies the robot pose - use one of
     *                              the odometry classes to provide this.
     * @param follower              The RAMSETE follower used to follow the trajectory.
     * @param kinematics            The kinematics for the robot drivetrain.
     * @param outputMetersPerSecond A function that consumes the computed left and right
     *                              wheel speeds.
     */
    public RamseteCommandRe(Trajectory trajectory,
                            Supplier<Pose2d> pose,
                            RamseteController follower,
                            DifferentialDriveKinematics kinematics,
                            BiConsumer<Double, Double> outputMetersPerSecond,
                            Telemetry telemetry) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_follower = follower;
        m_kinematics = kinematics;
        m_output = outputMetersPerSecond;

        m_feedforward = null;
        m_speeds = null;
        m_leftController = null;
        m_rightController = null;

        m_usePID = false;

        m_timer = new ElapsedTime();
        m_telemetry = telemetry;
    }

    @Override
    public void initialize() {
        m_prevTime = 0;
        Trajectory.State initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter
                                * initialState.velocityMetersPerSecond));
        m_timer.reset();
        if (m_usePID) {
            m_leftController.reset();
            m_rightController.reset();
        }
    }

    @Override
    public void execute() {
        double curTime = m_timer.seconds();
        double dt = curTime - m_prevTime;

        DifferentialDriveWheelSpeeds targetWheelSpeeds = m_kinematics.toWheelSpeeds(
                m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        if (m_usePID) {
            double leftFeedforward =
                    m_feedforward.calculate(leftSpeedSetpoint,
                            (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward =
                    m_feedforward.calculate(rightSpeedSetpoint,
                            (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

            leftOutput = leftFeedforward
                    + m_leftController.calculate(m_speeds.get().leftMetersPerSecond,
                    leftSpeedSetpoint);

            rightOutput = rightFeedforward
                    + m_rightController.calculate(m_speeds.get().rightMetersPerSecond,
                    rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }

        m_output.accept(leftOutput, rightOutput);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;

        m_telemetry.addData("Traj State",
                m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));
    }

    @Override
    public boolean isFinished() {
        return m_trajectory.getTotalTimeSeconds() < m_timer.seconds();
    }

    @Override
    public void end(boolean interrupted) {
        m_output.accept(0.0, 0.0);
    }
}