package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CapstoneDetector {

    private OpenCvCamera camera;
    private String cameraName;
    private CapstonePipeline capstonePipeline;
    private HardwareMap hardwareMap;

    public CapstoneDetector(HardwareMap hMap, String camName){
        hardwareMap = hMap;
        cameraName = camName;
    }

    public void init(){
        int cameraViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, cameraName), cameraViewId);

        capstonePipeline = new CapstonePipeline();

        camera.setPipeline(capstonePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public OpenCvCamera getCamera() {
        return camera;
    }
}
