package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.CapstonePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CapstoneDetector {

    private OpenCvCamera camera;
    private String cameraName;
    private CapstonePipeline capstonePipeline;
    private HardwareMap hardwareMap;
    private int width, height;

    public CapstoneDetector(HardwareMap hMap, String camName) {
        hardwareMap = hMap;
        cameraName = camName;
        width = 320;
        height = 240;
    }

    public CapstoneDetector(HardwareMap hMap, String camName, int width, int height) {
        hardwareMap = hMap;
        cameraName = camName;
        this.width = width;
        this.height = height;
    }

    public void init() {
        int cameraViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, cameraName), cameraViewId);

        capstonePipeline = new CapstonePipeline();

        camera.setPipeline(capstonePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                RobotLog.addGlobalWarningMessage("Dang the camera isn't working lol Error:" + errorCode);
            }
        });
    }

    //Todo: tune these values they are just estimations for now
    public Placement getPlacement() {
        if (capstonePipeline.getCentroid().x > width - 100)
            return Placement.RIGHT;
        else if (capstonePipeline.getCentroid().x < width - 200)
            return Placement.LEFT;
        else
            return Placement.CENTER;
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public enum Placement {
        LEFT,
        RIGHT,
        CENTER
    }
}
