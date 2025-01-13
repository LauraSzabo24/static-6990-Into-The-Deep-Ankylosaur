package Autonomous.Meet1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import Autonomous.Mailbox;

import org.firstinspires.ftc.teamcode.drive.Unused.DI_MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TrashyRed extends LinearOpMode {
    //region EXTENDER FLIPPER CONTROLS
    public static double ticksPerDegree = 537.7;
    private PIDController flp;
    public static double flpP = 0.004, flpI = 0.001, flpD = 0.00014, flpF = 0.001;
    public static int flpTarget;
    private PIDController ext;
    public static double extP = 0.004, extI = 0.00, extD = 0.00015, extF = 0.003;
    public static int extTarget;
    public static int divider = 1;
    FtcDashboard dashboard;
    //endregion
    //region DRIVER A MATERIAL
    IMU imu;
    IMU.Parameters parameters;
    DI_MecanumDrive drive;
    Pose2d poseEstimate;
    private double speed;
    private double multiply;
    //endregion

    //region DRIVER B MATERIAL
    private Servo wristLServo, wristRServo, spinnerServo, clawLServo, clawRServo;
    DcMotorEx flipMotor, armMotor;
    boolean clawIH;
    boolean pickupTwo = false;
    ElapsedTime jerkTimer = new ElapsedTime();
    boolean jerked = false;
    //endregion
    @Override
    public void runOpMode() throws InterruptedException {
        sleep(20);

        //region INITS
        //PID
        flp = new PIDController(flpP, flpI, flpD);
        ext = new PIDController(extP, extI, extD);

        //arm motors
        flipMotor = hardwareMap.get(DcMotorEx.class, "flip");
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //drive motors
        drive = new DI_MecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Mailbox.currentPose);

        //servos
        wristLServo = hardwareMap.get(Servo.class, "wristL");
        wristRServo = hardwareMap.get(Servo.class, "wristR");
        spinnerServo = hardwareMap.get(Servo.class, "spinner");
        clawLServo = hardwareMap.get(Servo.class, "clawL");
        clawRServo = hardwareMap.get(Servo.class, "clawR");

        //endregion

        //region TRAJECTORIES
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        //endregion

        //region RIGHT
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(15)
                /*.forward(16)
                .strafeLeft(20)
                .addDisplacementMarker(() -> {
                    divider = 4;
                    jerkTimer.reset();
                    flpTarget = -740;
                    for (int i = 0; i < 100; i++) {
                        flpCONTROLLER();
                    }
                    extTarget = 1500;
                    for (int i = 0; i < 100; i++) {
                        extCONTROLLER();
                    }
                    spinnerServo.setPosition(0.6989);
                    wristRServo.setPosition(0.555);
                    wristLServo.setPosition(0.2444);
                })*/
                /*.back(5)
                .strafeLeft(20)
                .back(20)*/
                .build();
        //endregion

        waitForStart();
        telemetry.setMsTransmissionInterval(50);
        if(isStopRequested()) return;
        Mailbox mail = new Mailbox();

        //DRIVING
        drive.setPoseEstimate(startPose);
        clawLServo.setPosition(0.6);
        clawRServo.setPosition(0.2);
        drive.followTrajectorySequence(right);
        mail.setAutoEnd((new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(-180))));
    }
    public void movementInit()
    {
        wristRServo.setPosition(0);
        wristLServo.setPosition(0.8);

        clawLServo.setPosition(0.2);
        clawRServo.setPosition(0.6);

        spinnerServo.setPosition(0.1522);
        flpTarget = -200;
        extTarget = 0;
    }
    public void extCONTROLLER()
    {
        ext.setPID(extP, extI, extD);
        int extPose = armMotor.getCurrentPosition();
        double extPwr = ext.calculate(extPose, extTarget) + (Math.cos(Math.toRadians(extTarget/ticksPerDegree)) * extF);
        armMotor.setPower(extPwr *(1/3.0));

        telemetry.addData("extPos ", extPose);
        telemetry.addData("extTarget ", extTarget);
    }
    public void flpCONTROLLER()
    {
        flp.setPID(flpP, flpI, flpD);
        int flpPose = flipMotor.getCurrentPosition();
        double flpPwr = flp.calculate(flpPose, flpTarget) + Math.cos(Math.toRadians(flpTarget/ticksPerDegree)) * flpF;
        flipMotor.setPower(flpPwr * (1/divider));

        telemetry.addData("flpPos ", flpPose);
        telemetry.addData("flpTarget ", flpTarget);
    }

}