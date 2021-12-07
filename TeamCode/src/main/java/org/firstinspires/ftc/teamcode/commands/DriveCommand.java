package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private DriveSubsystem driveSystem;
    private TankDriveSubsystem driveSystemRR;
    private DoubleSupplier fSpeed, tSpeed;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        driveSystem = driveSubsystem;
        fSpeed = forwardSpeed;
        tSpeed = turnSpeed;

        addRequirements(driveSystem);
    }

    public DriveCommand(TankDriveSubsystem driveSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        driveSystemRR = driveSubsystem;
        fSpeed = forwardSpeed;
        tSpeed = turnSpeed;

        addRequirements(driveSystemRR);
    }

    @Override
    public void execute() {
        if (driveSystemRR != null)
            driveSystemRR.drive(fSpeed.getAsDouble(), tSpeed.getAsDouble() * 0.8);
        else
            driveSystem.drive(fSpeed.getAsDouble(), tSpeed.getAsDouble() * 0.8);
    }
}
