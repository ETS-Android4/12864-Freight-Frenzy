package org.firstinspires.ftc.teamcode.motorTuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

@Config
public class MotorPIDCommand extends CommandBase {

    private MotorPIDSubsystem pidSubsystem;
    private Motor motor;

    public static double kP = 0.005;
    public static int targetPos = 200;

    public MotorPIDCommand(MotorPIDSubsystem subsystem){
        pidSubsystem = subsystem;
        motor = pidSubsystem.getMotor();

        addRequirements(pidSubsystem);
    }

    @Override
    public void initialize(){
        motor.setPositionCoefficient(kP);
        motor.setPositionTolerance(10.0);
        motor.setTargetPosition(targetPos);
        motor.resetEncoder();
    }

    @Override
    public void execute(){
        pidSubsystem.spin();
    }

    @Override
    public void end(boolean interrupt){
        pidSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return motor.atTargetPosition();
    }
}
