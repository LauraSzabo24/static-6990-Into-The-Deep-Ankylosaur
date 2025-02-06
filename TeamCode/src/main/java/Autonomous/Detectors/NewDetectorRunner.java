package Autonomous.Detectors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class NewDetectorRunner extends LinearOpMode {
    OpenCvCamera cam;
    int blockDistance;
    @Override
    public void runOpMode() throws InterruptedException {
        /*
        //region CAMERA JUNK
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());

        WebcamName camera = hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId);

        VerticalLineDetector redDetector = new VerticalLineDetector(telemetry);
        cam.setPipeline(redDetector);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("CAMERA WORKS");
                telemetry.update();
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("THE CAMERA DID NOT OPEN PROPERLY SEND HELP", errorCode);
                telemetry.update();
            }
        });

        sleep(20);
        //endregion
         */

        //redDetector.takePicture();
        blockDistance = 456;

        waitForStart();
        //convert blockdistance to extdistance
        //move ext to extdistance
        //meanwhile count changes in color
        //make sure they line up

        //do a full 360 spin and record colors in circle
        //find block-sized consistent section
        // if one go to perpendicular
        //if more than one
        //if two opposite side -> perpendicular
        //if not try one see if it picks else try again

        //retract calculate celebrate
    }
}