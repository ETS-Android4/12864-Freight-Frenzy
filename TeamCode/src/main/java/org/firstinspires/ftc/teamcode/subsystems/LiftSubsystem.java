package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

public class LiftSubsystem extends ProfiledPIDSubsystem {
    private Motor liftMotor;
    private static double kS = 1.0, kG = 1.0, kV = 1.0, kA = 1.0;
    private final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(kS, kG, kV, kA);
    private static double kP = 1.0, kI = 0, kD = 0;
    private final double distancePerPulse = Math.PI * 0.05 / 537.7;

    public LiftSubsystem(Motor liftMotor) {
        super(new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(5.0, 3.0)), 0.0);
        this.liftMotor = liftMotor;
        liftMotor.encoder.setDistancePerPulse(distancePerPulse);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setPoint) {
        double feedForward = elevatorFeedForward.calculate(setPoint.velocity);
        liftMotor.set(((output + feedForward) / distancePerPulse) / liftMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }

    @Override
    protected double getMeasurement() {
        return liftMotor.encoder.getDistance();
    }

    public void setMotor(){
        liftMotor.set(0.5);
    }
}