package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

@Config
public class DropOffSubsystem extends SubsystemBase {
    private final SimpleServo dropOffRight;
    private final SimpleServo dropOffLeft;

    public static double startLeft = 0.35, startRight = 0.5;
    public static double endLeft = -0.5, endRight = 0.9;


    public DropOffSubsystem(SimpleServo dropOffLeft, SimpleServo dropOffRight) {
        this.dropOffLeft = dropOffLeft;
        this.dropOffRight = dropOffRight;
    }

    public void drop() {
        dropOffLeft.setPosition(endLeft);
        dropOffRight.setPosition(endRight);
    }

    public void returnHome() {
        dropOffRight.setPosition(startRight);
        dropOffLeft.setPosition(startLeft);
    }
}
