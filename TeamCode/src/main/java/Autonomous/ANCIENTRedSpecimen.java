package Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class ANCIENTRedSpecimen extends OpMode {
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

    public static double flpPP = 10, flpPI = 0.05, flpPD = 0;
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
    IMU imu;
    IMU.Parameters parameters;
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
    private enum poseControlState
    {
        FREE,
        PICKUP,
        VARIPICKUP,
        NEWWALL,
        OLDWALL,
        LOW,
        HIGH,
        HOME,
        LOWBASKET,
        HIGHBASKET

    }
    private enum speedControlState
    {
        NORMAL,
        PRECISION,
        SUPERSPEED
    }
    private enum hangControlState
    {
        PHASEONE,
        PHASETWO,
        CONFIRMATION,
        DISENGAGE
    }
    speedControlState gameModeA;
    speedControlState gameModeB;
    hangControlState hangState;
    poseControlState controlState;
    boolean editMode = true;
    boolean notNormalLimits = false;
    boolean variablePickup = false;
    public int flpLowLimit;
    public int flpHighLimit;
    //endregion

    //region TRAJECTORIES
    TrajectorySequence preload, inventoryPickup, inventoryCycle, pickup, onePickup, oneCycle, twoPickup, twoCycle, threePickup, threeCycle;
    //endregion

    @Override
    public void init()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Mailbox mail = new Mailbox();
        hardwareInit();
        Pose2d startPose = new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);
        movementInitI();

        //region PRELOAD
        preload = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(16, 0))
                .waitSeconds(0.001)
                .lineTo(new Vector2d(30, 0), NewMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(60))
                .waitSeconds(10)
                .addTemporalMarker(0,() -> {
                    flpPosTarget = 0;
                    extTarget = 1300;
                    spin.setPosition(0.1567);
                    bigWristR.setPosition(0.0389);
                    bigWristL.setPosition(0.9589);
                    smallWrist.setPosition(0.257);
                })
                .addTemporalMarker(1.5,() -> {
                    extTarget = 650;
                })
                .addTemporalMarker(2.1,() -> {
                    claw.setPosition(0.3);
                })
                .addTemporalMarker(2.2,() -> {drive.followTrajectorySequenceAsync(inventoryPickup, mail);})
                .build();
        //endregion

        //region INVENTORY PICKUP
        inventoryPickup = drive.trajectorySequenceBuilder(preload.end())
                .lineTo(new Vector2d(20, 0))
                .splineToConstantHeading(new Vector2d(14, -32), 0, NewMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(60))
                .lineTo(new Vector2d(0, -32))
                .waitSeconds(10)
                .addTemporalMarker(0,() -> {
                    extTarget = 0;
                    flpPosTarget = 0;
                    spin.setPosition(0.7111);
                    smallWrist.setPosition(0.1367);
                    bigWristL.setPosition(0.95);
                    bigWristR.setPosition(0.05);
                })
                .addTemporalMarker(1,() -> {
                    extTarget = 0;
                    flpPosTarget = 0;
                    spin.setPosition(0.7111);
                    smallWrist.setPosition(0.5667);
                    bigWristR.setPosition(0.8094);
                    bigWristL.setPosition(0.1878);
                })
                .addTemporalMarker(2.4,() -> {
                    claw.setPosition(1);//0.6
                })
                .addTemporalMarker(2.5,() -> {
                    smallWrist.setPosition(0.4633);
                    bigWristR.setPosition(0.7489);
                    bigWristL.setPosition(0.2478);
                })
                .addTemporalMarker(2.7,() -> {drive.followTrajectorySequenceAsync(inventoryCycle, mail);})
                .build();
        //endregion

        //region INVENTORY CYCLE
        inventoryCycle = drive.trajectorySequenceBuilder(inventoryPickup.end())
                .splineToConstantHeading(new Vector2d(23, 3), 0, NewMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(30))
                .lineTo(new Vector2d(29, 3), NewMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(20))
                .waitSeconds(2)
                .lineTo(new Vector2d(28, 0), NewMecanumDrive.getVelocityConstraint(90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(100))
                .addTemporalMarker(0.5,() -> {
                    flpPosTarget = 0;
                    extTarget = 1370;
                    spin.setPosition(0.1567);
                    bigWristR.setPosition(0.0389);
                    bigWristL.setPosition(0.9589);
                    smallWrist.setPosition(0.257);
                })
                .addTemporalMarker(2.4,() -> {
                    extTarget = 650;
                })
                .addTemporalMarker(2.7,() -> {
                    claw.setPosition(0.3);
                })
                .addTemporalMarker(2.8,() -> {drive.followTrajectorySequenceAsync(pickup, mail);})
                .build();
        //endregion6

        //region BAD PICKUP
        pickup = drive.trajectorySequenceBuilder(inventoryCycle.end())
                .addTemporalMarker(0,() -> {
                    extTarget = 0;
                    flpPosTarget = 0;
                    spin.setPosition(0.7111);
                    smallWrist.setPosition(0.1367);
                    bigWristL.setPosition(0.95);
                    bigWristR.setPosition(0.05);
                })
                .lineTo(new Vector2d(20, 0))
                .splineToConstantHeading(new Vector2d(20, -22),0)
                .splineToConstantHeading(new Vector2d(30, -22), 0)
                .splineToConstantHeading(new Vector2d(48, -36),0)
                .splineToConstantHeading(new Vector2d(10, -38),0, NewMecanumDrive.getVelocityConstraint(90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(70))

                .splineToConstantHeading(new Vector2d(36, -42),0)
                .splineToConstantHeading(new Vector2d(50, -52),0, NewMecanumDrive.getVelocityConstraint(90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(70))
                .splineToConstantHeading(new Vector2d(10, -52),0)

                .splineToConstantHeading(new Vector2d(36, -56),0)
                .splineToConstantHeading(new Vector2d(50, -60),0, NewMecanumDrive.getVelocityConstraint(90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(70))
                .splineToConstantHeading(new Vector2d(10, -60), 0)
                .build();
//endregion

        //region CYCLES
        oneCycle = drive.trajectorySequenceBuilder(pickup.end())
                .splineToConstantHeading(new Vector2d(16, 3), 0, NewMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(30))
                .lineTo(new Vector2d(26, 3), NewMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(10)
                .addTemporalMarker(0,() -> {
                    flpPosTarget = 0;
                    extTarget = 1270;
                    spin.setPosition(0.1567);
                    bigWristR.setPosition(0.0389);
                    bigWristL.setPosition(0.9589);
                    smallWrist.setPosition(0.257);
                })
                .addTemporalMarker(3,() -> {
                    extTarget = 650;
                })
                .addTemporalMarker(3.5,() -> {
                    claw.setPosition(0.3);
                })
                //.addTemporalMarker(5,() -> {drive.followTrajectorySequenceAsync(twoPickup, mail);})
                .build();
        //endregion
        drive.followTrajectorySequenceAsync(preload, mail);
        mail.setAutoEnd((new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())));
    }

    public void miniPickup()
    {
        spin.setPosition(0.1767);
        smallWrist.setPosition(0.6067);
        bigWristR.setPosition(0.01);
        bigWristL.setPosition(0.99);
          /*
            AUTO PICKUP
            bigL - 0.99
            bigR - 0.01
            small - 0.6067
            spin - 0.1767
             */
    }
    public void jerk()
    {
        telemetry.addLine("JERK");
        extTarget = extTarget - 600;
        jerkTimer.reset();
        while (jerkTimer.time() < 0.7) {
            extOldCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
        extTarget = 908;
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
        bigWristL.setPosition(0.2078);
        bigWristR.setPosition(0.76);
        smallWrist.setPosition(0.1367);
        claw.setPosition(0.7);
        spin.setPosition(0.725);
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
        telemetry.addData("FLIP VELO", velocityVal);
        if(velocityVal>1700){
            velocityVal=1700;
        }
        else if (velocityVal<-1700)
        {
            velocityVal = -1700;
        }
        flipMotor.setVelocity(velocityVal);
    }
    public void RAINBOW(double speed)
    {
        for(double i=0; i<1; i+=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=1; i>0; i-=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(i, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=0; i<1; i+=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 1, i, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=1; i>0; i-=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, i, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=0; i<1; i+=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(i, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=1; i>0; i-=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 0, i, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}

//region INTAKES
        /*
        intakeOne = drive.trajectorySequenceBuilder(inventoryCycle.end())
                .lineTo(new Vector2d(20, 0))
                .splineToConstantHeading(new Vector2d(20, -39), 0, NewMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(60))
                .lineTo(new Vector2d(25, -39), NewMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(20))
                .waitSeconds(1)
                .lineTo(new Vector2d(0, -32))
                .addTemporalMarker(0,() -> {
                    extTarget = 240;
                    flpPosTarget = 0;
                    claw.setPosition(0.3);
                    spin.setPosition(0.1767);
                    smallWrist.setPosition(0.1367);
                    bigWristL.setPosition(0.95);
                    bigWristR.setPosition(0.05);
                })
                .addTemporalMarker(1,() -> {
                    extTarget = 240;
                    claw.setPosition(0.3);
                    spin.setPosition(0.1767);
                    smallWrist.setPosition(0.6761);
                    bigWristR.setPosition(0.09);
                    bigWristL.setPosition(0.91);
                })

                .addTemporalMarker(1.5,() -> {
                    extTarget = 100;
                })
                .addTemporalMarker(1.9,() -> {
                    claw.setPosition(1);
                })
                .addTemporalMarker(2.3,() -> {
                    extTarget = 0;
                    flpPosTarget = 0;
                    spin.setPosition(0.7111);
                    smallWrist.setPosition(0.5667);
                    bigWristR.setPosition(0.8094);
                    bigWristL.setPosition(0.1878);
                })
                .addTemporalMarker(3.6,() -> {
                    claw.setPosition(0.3);
                })
                .build();

        intakeTwo = drive.trajectorySequenceBuilder(intakeOne.end())
                .lineTo(new Vector2d(20, 0))
                .splineToConstantHeading(new Vector2d(20, -41), 0, NewMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(60))
                .lineTo(new Vector2d(32, -41), NewMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(20))
                .waitSeconds(0.1)
                .lineTo(new Vector2d(0, -32))
                .waitSeconds(10)
                .addTemporalMarker(0,() -> {
                    extTarget = 240;
                    flpPosTarget = 0;
                    claw.setPosition(0.3);
                    spin.setPosition(0.1767);
                    smallWrist.setPosition(0.1367);
                    bigWristL.setPosition(0.95);
                    bigWristR.setPosition(0.05);
                })
                .addTemporalMarker(1.3,() -> {
                    extTarget = 240;
                    claw.setPosition(0.3);
                    spin.setPosition(0.1767);
                    smallWrist.setPosition(0.6067);
                    bigWristR.setPosition(0.01);
                    bigWristL.setPosition(0.99);
                })

                .addTemporalMarker(2,() -> {
                    extTarget = 0;
                })
                .addTemporalMarker(2.5,() -> {
                    claw.setPosition(1);
                })
                .addTemporalMarker(2.6,() -> {
                    extTarget = 160;
                })
                .addTemporalMarker(2.8,() -> {
                    extTarget = 0;
                    flpPosTarget = 0;
                    spin.setPosition(0.7111);
                    smallWrist.setPosition(0.5667);
                    bigWristR.setPosition(0.8094);
                    bigWristL.setPosition(0.1878);
                })
                .addTemporalMarker(3.6,() -> {
                    claw.setPosition(0.3);
                })
                .build();*/
//endregion

//region BAD PICKUP
        /*pickup = drive.trajectorySequenceBuilder(inventoryCycle.end())
                .addTemporalMarker(0,() -> {
                    extTarget = 0;
                    flpPosTarget = 0;
                    spin.setPosition(0.7111);
                    smallWrist.setPosition(0.1367);
                    bigWristL.setPosition(0.95);
                    bigWristR.setPosition(0.05);
                })
                .lineTo(new Vector2d(20, 0), NewMecanumDrive.getVelocityConstraint(90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(100))
                .splineToConstantHeading(new Vector2d(20, -22),0)
                .lineTo(new Vector2d(30, -22))
                .splineToConstantHeading(new Vector2d(48, -36),0)
                .lineTo(new Vector2d(10, -32), NewMecanumDrive.getVelocityConstraint(90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(100))

                .lineTo(new Vector2d(36, -42))
                .splineToConstantHeading(new Vector2d(48, -52),0, NewMecanumDrive.getVelocityConstraint(90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(100))
                .lineTo(new Vector2d(10, -52))

                .lineTo(new Vector2d(36, -53))
                .splineToConstantHeading(new Vector2d(48, -56),0, NewMecanumDrive.getVelocityConstraint(90, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(100))
                .lineTo(new Vector2d(10, -56))
                .build();*/
//endregion