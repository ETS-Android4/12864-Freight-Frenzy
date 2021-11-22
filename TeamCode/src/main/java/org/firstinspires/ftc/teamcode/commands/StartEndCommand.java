package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

public class StartEndCommand extends CommandBase {

    private final Runnable init, endRun;

    public StartEndCommand(Runnable toRun, Runnable toEnd, Subsystem... requirements) {
        init = toRun;
        endRun = toEnd;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        init.run();
    }

    @Override
    public void end(boolean interrupted) {
        endRun.run();
    }
}
