package Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Red4Specimen extends OpMode {
    NewMecanumDrive drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    //region LED EFFECTS
    Gamepad.LedEffect resetField = new Gamepad.LedEffect.Builder()
            .addStep(0.5, 0.5, 0.5, 150)
            .addStep(0, 0, 0, 150)
            .addStep(0.5, 0.5, 0.5, 150)
            .addStep(0, 0, 0, 150)
            .addStep(0.5, 0.5, 0.5, 150)
            .addStep(0, 0, 0, 150)
            .build();
    ElapsedTime flashTimer = new ElapsedTime();

    //endregion

    //region FLIPPER CONTROLLER
    ElapsedTime timer = new ElapsedTime();
    private double flpPosError = 0;
    private double flpPosISum = 0;

    public static double flpPP = 2.5, flpPI = 0, flpPD = 0;
    public static int flpPosTarget = 0;
    //endregion

    //region EXTENDER CONTROLLER
    //POSITION
    private PIDController ext;
    //public static double extP = 0.005, extI = 0.03, extD = 0;
    public static double extP = 0.001, extI = 0.03, extD = 0;
    public static int extTarget;
    //VELOCITY
    ElapsedTime velTimer = new ElapsedTime();
    private double extVeloError = 0;
    private double extVeloISum = 0;
    public static double extVP = 0.001, extVI = 0, extVD = 0;
    public static int extVeloTarget = 0;
    //endregion

    //region DRIVER A MATERIAL
    Pose2d poseEstimate;
    private double speed;
    private double multiply;
    double oldTime = 0;
    double extPercentage;
    //endregion

    //region DRIVER B MATERIAL
    private Servo smallWrist, bigWristR, bigWristL, spin, claw;
    DcMotorEx flipMotor, extLMotor, extRMotor;
    boolean clawIH;
    ElapsedTime jerkTimer = new ElapsedTime();
    int spinnerPos = 0;
    int smallWristPos = 0;
    //endregion

    @Override
    public void init()
    {
        Mailbox mail = new Mailbox();
        hardwareInit();
        movementInitI();

        //region PRELOAD DROP OFF
       TrajectorySequence preload = drive.trajectorySequenceBuilder(new Pose2d())
               .splineTo(new Vector2d(29, 13), 0)
               .addTemporalMarker(0.5,() -> {
                    highPosition();
               })
                .build();
        //endregion

        //back to player
        //pickup x2
        //wall to place
        //wall to place
        //wall to place
        //drag and drop
        //wall to place
        //park

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.followTrajectorySequenceAsync(preload, mail);
        mail.setAutoEnd((new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(-180))));
    }

    public void highPosition()
    {
        telemetry.addLine("HIGH POSITION");
        if(flpPosTarget<200) {
            extTarget = 1160;
            flpPosTarget = 0;
            spin.setPosition(0.7106);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
        }
        else {
            extTarget = 0;
            jerkTimer.reset();
            while(-extLMotor.getCurrentPosition()>10 || jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            flpPosTarget = 0;
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
            jerkTimer.reset();
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.7106);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
            extTarget = 1160;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {

                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
        }
    }
    public void homePosition()
    {
        telemetry.addLine("HOME POSITION");
        if(flpPosTarget<200) {
            extTarget = 0;
            flpPosTarget = 0;
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
        }
        else {
            extTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            flpPosTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
        }

    }
    public void oldWallPosition()
    {
        telemetry.addLine("OLD WALL POSITION");
        if(flpPosTarget>1200) {
            extTarget = 0;
            flpPosTarget = 1650;
            spin.setPosition(0.1528);
            bigWristR.setPosition(0.48);
            bigWristL.setPosition(0.5183);
            smallWrist.setPosition(0.3206);
        }
        else {
            extTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            flpPosTarget = 1650;
            jerkTimer.reset();
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1528);
            bigWristR.setPosition(0.48);
            bigWristL.setPosition(0.5183);
            smallWrist.setPosition(0.3206);
        }

    }
    public void newWallPosition()
    {
        telemetry.addLine("NEW WALL POSITION");
        if(flpPosTarget<200) {
            extTarget = 0;
            flpPosTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.6094);
            bigWristL.setPosition(0.39);
            smallWrist.setPosition(0.5317);
            claw.setPosition(0.3);
        }
        else {
            extTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            flpPosTarget = 0;
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
            jerkTimer.reset();
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.6094);
            bigWristL.setPosition(0.39);
            smallWrist.setPosition(0.5317);
            claw.setPosition(0.3);
        }
    }
    public void pickupPosition()
    {
        telemetry.addLine("PICKUP POSITION");
        if(flpPosTarget>1200) {
            extTarget = 40;
            flpPosTarget = 1650;
            spin.setPosition(0.1528);
            bigWristR.setPosition(0.39);
            bigWristL.setPosition(0.6078);
            smallWrist.setPosition(0.3206);
        }
        else {
            extTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
            flpPosTarget = 1650;
            jerkTimer.reset();
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1528);
            bigWristR.setPosition(0.39);
            bigWristL.setPosition(0.6078);
            smallWrist.setPosition(0.3206);
            extTarget = 40;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
        }
    }
    public void jerk()
    {
        telemetry.addLine("JERK");
        if(extTarget>=300) {
            extTarget = extTarget + 300;
            jerkTimer.reset();
            while (jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
        }
    }

    @Override
    public void loop()  {
        drive.update();
        telemetry.addData("CURR HEADING", drive.getPoseEstimate().getHeading());
        telemetry.update();
        drive.update();
        poseEstimate = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());
        extOldCONTROLLER();
        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
    }
    public void hardwareInit()
    {
        //RANDOM
        dashboard = FtcDashboard.getInstance();

        //PID
        ext = new PIDController(extP, extI, extD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //arm motors
        flipMotor = hardwareMap.get(DcMotorEx.class, "FLIP");
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extLMotor = hardwareMap.get(DcMotorEx.class, "EL");
        extLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extRMotor = hardwareMap.get(DcMotorEx.class, "ER");
        extRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //drive motors
        drive = new NewMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.reverseMotors();
        drive.setPoseEstimate(Mailbox.currentPose);

        //servos
        smallWrist = hardwareMap.get(Servo.class, "SW");
        bigWristL = hardwareMap.get(Servo.class, "BWL");
        bigWristR = hardwareMap.get(Servo.class, "BWR");
        spin = hardwareMap.get(Servo.class, "SPIN");
        claw = hardwareMap.get(Servo.class, "CLAW");
    }
    public void movementInitI()
    {
        extTarget = 0;
        bigWristL.setPosition(0.13);
        bigWristR.setPosition(0.87);
        smallWrist.setPosition(0.1519);
        claw.setPosition(0.6);
        spin.setPosition(0.1528);
    }
    public void extOldCONTROLLER()
    {
        ext.setPID(extP, extI, extD);
        int extPose = -extLMotor.getCurrentPosition();
        double extPwr = ext.calculate(extPose, extTarget);
        extLMotor.setPower(-extPwr);
        extRMotor.setPower(extPwr);

        telemetry.addData("extPos ", extPose);
        telemetry.addData("extTarget ", extTarget);
    }
    public void flpCONTROLLER(int target, int state) //in with the target -> out with the velocity
    {
        int currError = target - state;
        double time = timer.seconds();
        timer.reset();
        flpPosISum += currError * time;
        double deriv = (currError - flpPosError)/time;
        flpPosError = currError;

        double velocityVal = (flpPP * currError) + (flpPI * flpPosISum) + (flpPD*deriv);
        flipMotor.setVelocity(velocityVal);
    }
}