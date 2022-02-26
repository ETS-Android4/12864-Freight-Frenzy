package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private DriveSubsystem driveSystem;
    private TankDriveSubsystem driveSystemRR;
    private DoubleSupplier fSpeed, tSpeed, multiplier;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        driveSystem = driveSubsystem;
        fSpeed = forwardSpeed;
        tSpeed = turnSpeed;
        multiplier = () -> 1.0;

        addRequirements(driveSystem);
    }

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, DoubleSupplier mult) {
        driveSystem = driveSubsystem;
        fSpeed = forwardSpeed;
        tSpeed = turnSpeed;
        multiplier = mult;

        addRequirements(driveSystem);
    }

    public DriveCommand(TankDriveSubsystem driveSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        driveSystemRR = driveSubsystem;
        fSpeed = forwardSpeed;
        tSpeed = turnSpeed;
        multiplier = () -> 1.0;


        addRequirements(driveSystemRR);
    }

    @Override
    public void execute() {
        if (driveSystemRR != null)
            driveSystemRR.drive(fSpeed.getAsDouble(), tSpeed.getAsDouble() * 0.8);
        else
            driveSystem.drive(fSpeed.getAsDouble() * multiplier.getAsDouble(), tSpeed.getAsDouble() * 0.75 * multiplier.getAsDouble());
    }
}
