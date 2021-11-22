package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class LiftSubsystemNoPID extends SubsystemBase {

    private final Motor motor;

    public LiftSubsystemNoPID(Motor liftMotor) {
        motor = liftMotor;
    }

    public void motorUp() {
        motor.set(0.7);
    }

    public void motorDown() {
        motor.set(-0.4);
    }

    public void motorStop() {
        motor.stopMotor();
    }
}
