package Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import TeleOp.RedTele;

@Autonomous
public class Red5Specimen extends OpMode {
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
    //region CONTROL STATE
    private enum poseControlState {
        FREE,
        PICKUP,
        NEWWALL,
        OLDWALL,
        LOW,
        HIGH,
        HOME,
        LOWBASKET,
        HIGHBASKET

    }
    poseControlState controlState;
    //endregion

    //region TRAJECTORIES
    TrajectorySequence pickUp, preload;
    //endregion

    @Override
    public void init()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Mailbox mail = new Mailbox();
        hardwareInit();
        movementInitI();
        Pose2d startPose = new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);

        //region PRELOAD & DROP OFF
        preload = drive.trajectorySequenceBuilder(startPose)
               .lineTo(new Vector2d(32, 13))
               .waitSeconds(1)
               .addTemporalMarker(0,() -> {
                   highPosition();
               })
               .addTemporalMarker(2,() -> {
                   jerk();
               })
               .addTemporalMarker(4,() -> {
                   homePosition();
               })
                .lineTo(new Vector2d(20, 5))
                .splineToConstantHeading(new Vector2d(30, -33), Math.toRadians(0))
                .addTemporalMarker(4,() -> {
                    miniPickup();
                })
                .addTemporalMarker(4.5,() -> {
                    claw.setPosition(0.3);
                })
                //.addTemporalMarker(6,() -> {drive.followTrajectorySequenceAsync(pickUp, mail);})
               .build();
        //endregion

        drive.followTrajectorySequenceAsync(preload, mail);
        mail.setAutoEnd((new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())));
    }

    public void miniPickup()
    {
        spin.setPosition(0.1572);
        smallWrist.setPosition(0.6828);
        bigWristR.setPosition(0.8);
        bigWristL.setPosition(0.2);
    }
    public void highPosition()
    {
        telemetry.addLine("HIGH POSITION");
        controlState = poseControlState.HIGH;
        if(flpPosTarget<200) {
            extTarget = 1100; //1160
            flpPosTarget = 0;
            spin.setPosition(0.1567);
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
            extTarget = 1100;
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
        controlState = poseControlState.HOME;
        if(flpPosTarget<200) {
            extTarget = 0;
            flpPosTarget = 0;
            if(Math.abs(extLMotor.getCurrentPosition())>1200){
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.3194);
                bigWristL.setPosition(0.68);
                smallWrist.setPosition(0.6372);
                while(jerkTimer.time() < 0.7) {
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
        }
        else {
            extTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3 && Math.abs(extLMotor.getCurrentPosition())<50) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
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
        }
    }
    public void oldWallPosition()
    {
        telemetry.addLine("OLD WALL POSITION");
        controlState = poseControlState.OLDWALL;
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
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.3194);
            bigWristL.setPosition(0.68);
            smallWrist.setPosition(0.6372);
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
        controlState = poseControlState.NEWWALL;
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
        controlState = poseControlState.PICKUP;
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
        extTarget = extTarget - 400;
        jerkTimer.reset();
        while (jerkTimer.time() < 1.5) {
            extOldCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
        claw.setPosition(0.3);
    }

    @Override
    public void loop()  {
        drive.update();
        poseEstimate = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());
        extOldCONTROLLER();
        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        telemetry.addData("CURR HEADING", drive.getPoseEstimate().getHeading());
        telemetry.addData("XPos", poseEstimate.getX());
        telemetry.addData("YPos", poseEstimate.getY());
        telemetry.update();

        if(controlState == poseControlState.PICKUP)
        {
            extPercentage = (extTarget-40.0)/1120;
            spin.setPosition(0.1528);
            smallWrist.setPosition(0.1706 + ((0.2556-0.1706)*extPercentage));
            bigWristL.setPosition(0.6989 + ((0.7389-0.6989)*extPercentage));
            bigWristR.setPosition(0.3 - ((0.3-0.26)*extPercentage));
            //EXT          40         1120
            //smallWrist - 0.1706 - 0.2556
            //bigWristL - 0.6989 - 0.7389
            //bigWristR - 0.3 - 0.26
        }
        //endregion
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