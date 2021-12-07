package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

public class DropOffSubsystem extends SubsystemBase {
    private final SimpleServo dropOffRight;
    private final SimpleServo dropOffLeft;

    public DropOffSubsystem(SimpleServo dropOffLeft, SimpleServo dropOffRight) {
        this.dropOffLeft = dropOffLeft;
        this.dropOffRight = dropOffRight;
    }

    public void drop() {
        dropOffLeft.setPosition(0.6);
        dropOffRight.setPosition(0.9);
    }

    public void returnHome() {
        dropOffRight.setPosition(0.5);
        dropOffLeft.setPosition(1.0);
    }
}
