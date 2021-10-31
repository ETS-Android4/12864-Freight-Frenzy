package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class SpinnerSubsystem extends SubsystemBase {

    private Motor duckSpinner;

    public SpinnerSubsystem(Motor spinner) {
        duckSpinner = spinner;
    }

    public void spin(double curSide) {
        duckSpinner.set(curSide);
    }

    public void stop() {
        duckSpinner.stopMotor();
    }
}
