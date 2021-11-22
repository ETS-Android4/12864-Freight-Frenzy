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
        dropOffLeft.turnToAngle(65.0);
//        dropOffRight.turnToAngle(-5.0);
    }

    public void returnHome() {
//        dropOffRight.turnToAngle(0.0);
        dropOffLeft.turnToAngle(0.0);
    }
}
