package org.firstinspires.ftc.teamcode.soMuchAuto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "which is which")
public class OneMotorAtATime extends CommandOpMode {

    private Motor frontLeft, backLeft, frontRight, backRight;

    @Override
    public void initialize() {
        frontLeft = new Motor(hardwareMap, "fL");
        backLeft = new Motor(hardwareMap, "bL");
        frontRight = new Motor(hardwareMap, "fR");
        backRight = new Motor(hardwareMap, "bR");

        schedule(new SequentialCommandGroup(new InstantCommand(() -> frontLeft.set(1.0))
                .andThen(new WaitCommand(4000).andThen(new InstantCommand(() -> frontLeft.stopMotor()))),
                new InstantCommand(() -> backLeft.set(1.0))
                        .andThen(new WaitCommand(4000).andThen(new InstantCommand(() -> backLeft.stopMotor()))),
                new InstantCommand(() -> frontRight.set(1.0))
                        .andThen(new WaitCommand(4000).andThen(new InstantCommand(() -> frontRight.stopMotor()))),
                new InstantCommand(() -> backRight.set(1.0))
                        .andThen(new WaitCommand(4000).andThen(new InstantCommand(() -> backRight.stopMotor())))));
    }
}
