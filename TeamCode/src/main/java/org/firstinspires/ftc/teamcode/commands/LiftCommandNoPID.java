package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystemNoPID;

public class LiftCommandNoPID extends CommandBase {

    private LiftSubsystemNoPID liftSubsystem;
    private ElapsedTime time;
    //you get none
    private int level = 0;
    private double timeToLift = 0.4;

    public static double toBottom = 1.35, toTier2 = 0.45, toTier3 = 1.5;

    public LiftCommandNoPID(LiftSubsystemNoPID liftSubsystemNoPID, ElapsedTime timer) {
        liftSubsystem = liftSubsystemNoPID;
        time = timer;
    }

    public LiftCommandNoPID(LiftSubsystemNoPID liftSubsystemNoPID, ElapsedTime timer, int levelSt) {
        liftSubsystem = liftSubsystemNoPID;
        time = timer;
        level = levelSt;
    }


    @Override
    public void initialize() {
        time.reset();
        if (level == 0) {
            liftSubsystem.motorUp();
            timeToLift = toTier2;
            level++;
        } else if (level == 1) {
            liftSubsystem.motorUp();
            timeToLift = toTier3;
            level++;
        } else {
            liftSubsystem.motorDown();
            timeToLift = toBottom;
            level = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return time.seconds() >= timeToLift;
    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.motorStop();
    }
}
