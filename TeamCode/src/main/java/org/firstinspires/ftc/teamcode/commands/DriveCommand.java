package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private DriveSubsystem driveSystem;
    private DoubleSupplier fSpeed, tSpeed;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        driveSystem = driveSubsystem;
        fSpeed = forwardSpeed;
        tSpeed = turnSpeed;

        addRequirements(driveSystem);
    }

    @Override
    public void execute() {
        driveSystem.drive(fSpeed.getAsDouble(), tSpeed.getAsDouble() * 0.9);
    }
}
