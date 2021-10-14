package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testttt")
public class DriveTestOpMode extends CommandOpMode {

    private Motor frontLeft, backLeft, frontRight, backRight;

    @Override
    public void initialize() {
        frontLeft = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_435);
        backLeft = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_435);
        frontRight = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_435);
        backRight = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_435);

        schedule(new SequentialCommandGroup(new InstantCommand(() -> frontLeft.set(1.0)).andThen(new WaitCommand(2000)),
                new InstantCommand(() -> backLeft.set(1.0)).andThen(new WaitCommand(2000)),
                new InstantCommand(() -> frontRight.set(1.0)).andThen(new WaitCommand(2000)),
                new InstantCommand(() -> backRight.set(1.0))));
    }
}
