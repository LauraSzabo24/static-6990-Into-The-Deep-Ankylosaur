package Autonomous.Detectors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Autonomous.Mailbox;

@Autonomous
public class DetectorRunner extends LinearOpMode {
    OpenCvCamera cam;
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, intakeMotor;
    private Servo clawServo, armLeftServo, armRightServo, intakeLift;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());

        WebcamName camera = hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId);

        OrientationDetector redDetector = new OrientationDetector(telemetry);
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

        redDetector.takePicture();
        while(!redDetector.doneFilming())
        {
            String printout = redDetector.getArea();
            telemetry.addLine(printout);
            if(printout != null)
            {
                telemetry.update();
            }
        }
        waitForStart();

        // region MOTORS AND SERVOS INIT STUFF
        // NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
        //endregion

        //region TRAJECTORIES (left/right in robot perspective)
        //Pose2d startPose = new Pose2d(-14, 0, Math.toRadians(-90)); //90
        //drive.setPoseEstimate(startPose);
        //endregion

        /*TrajectorySequence test = drive.trajectorySequenceBuilder(startPose) //(-14, 0_)
                .turn(Math.toRadians(123))
                .build();


        waitForStart();
        //PropDetectorBLUE.Location place = redDetector.getLocation();
        telemetry.setMsTransmissionInterval(50);
        if(isStopRequested()) return;
        Mailbox mail = new Mailbox();

        //DRIVING
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequence(test, mail);

        //drive.followTrajectorySequence(test, mail);
        mail.setAutoEnd((new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(-180))));
        */
    }
}