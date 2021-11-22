package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SpinnerSubsystem;

public class SpinnerCommand extends CommandBase {

    private final SpinnerSubsystem spinnerSubsystem;
    private boolean spinSide;

    public SpinnerCommand(SpinnerSubsystem subsystem, boolean side) {
        spinnerSubsystem = subsystem;
        spinSide = side;
        addRequirements(spinnerSubsystem);
    }

    @Override
    public void initialize() {
        spinnerSubsystem.spin(spinSide ? -0.25 : 0.25);
    }

    @Override
    public void end(boolean interrupted) {
        spinnerSubsystem.stop();
    }
}
