package org.firstinspires.ftc.teamcode.motorTuning;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorPIDSubsystem extends SubsystemBase {

    private Motor motor;
    private Telemetry tele;

    public MotorPIDSubsystem(Motor testMotor, Telemetry telemetry){
        motor = testMotor;
        tele = telemetry;
        motor.setRunMode(Motor.RunMode.PositionControl);
    }

    public void spin(){
        motor.set(0.5);
    }

    public void stop(){
        motor.stopMotor();
    }
    public Motor getMotor(){
        return motor;
    }

    @Override
    public void periodic() {
        tele.addData("Motor Position ", motor.getCurrentPosition());
        tele.update();
    }
}
