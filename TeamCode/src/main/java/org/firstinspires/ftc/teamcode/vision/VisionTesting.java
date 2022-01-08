package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "visionTesting")
public class VisionTesting extends CommandOpMode {

    private CapstoneDetector capstoneDetector;

    @Override
    public void initialize() {

        capstoneDetector = new CapstoneDetector(hardwareMap, "coolio");
        capstoneDetector.init();

        FtcDashboard.getInstance().startCameraStream(capstoneDetector.getCamera(), 30);

        schedule(new WaitUntilCommand(this::isStarted).andThen(new RunCommand(() -> {
            telemetry.addData("Capstone Placement", capstoneDetector.getPlacement());
            telemetry.addData("Lift Pos", capstoneDetector.placementId());
            telemetry.update();
        })));
    }
}
