package org.firstinspires.ftc.teamcode.soMuchAuto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.paths.FreightSideRed;
import org.firstinspires.ftc.teamcode.rr.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.rr.subsystems.TankDriveSubsystem;

@Autonomous(name = "Poggers")
public class RRAutoTesting extends CommandOpMode {

    private SampleTankDrive tankDrive;
    private TankDriveSubsystem tankDriveSubsystem;


    @Override
    public void initialize() {
        tankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(tankDrive);

        schedule(new WaitUntilCommand(this::isStarted).andThen(new FreightSideRed(tankDriveSubsystem)));

    }
}
