package TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.util.Log;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;

import java.io.File;
import java.io.IOException;

import Autonomous.Mailbox;

@TeleOp
@Config
public class RedTele extends LinearOpMode {
    //region FLIPPER CONTROLLER
    //POSITION
    ElapsedTime timer = new ElapsedTime();
    private double flpPosError = 0;
    private double flpPosISum = 0;

    public static double flpPP = 10, flpPI = 0, flpPD = 0;
    public static int flpPosTarget = 0;

    //VELOCITY
    private double flpVeloError = 0;
    private double flpVeloISum = 0;
    public static int flpVeloTarget = 0;
    public static int testTarget = 1200;
    public static double flpVP = 0.0002, flpVI = 0.004, flpVD = 0.0000001;  //RISING
    //endregion

    //region EXTENDER CONTROLLER
    public static double ticksPerDegree = 537.7;
    private PIDController ext;
    public static double extP = 0.004, extI = 0.00, extD = 0.00015, extF = 0.003;
    public static int extTarget;
    FtcDashboard dashboard;
    //endregion

    //region DRIVER A MATERIAL
    IMU imu;
    IMU.Parameters parameters;
    NewMecanumDrive drive;
    Pose2d poseEstimate;
    private double speed;
    private double multiply;
    //endregion

    //region DRIVER B MATERIAL
    private Servo clawServo;
    DcMotorEx flipMotor, armMotor;
    boolean clawIH;
    ElapsedTime jerkTimer = new ElapsedTime();
    double extLimit = 1600;
    double flpLimit = 4300;
    //endregion

    //region GAMEPADS
    Gamepad currG1;
    Gamepad oldG1;
    Gamepad currG2;
    Gamepad oldG2;

    //endregion

    //region CONTROL STATE
    private enum poseControlState
    {
        FREE,
        PICKUP,
        LOW,
        HIGH,
        HOME
    }
    private enum speedControlState
    {
        NORMAL,
        PRECISION,
        SUPERSPEED
    }
    speedControlState gameModeA;
    speedControlState gameModeB;
    poseControlState currentState;
    boolean notNormalLimits = false;

    //endregion

    //INITIALIZATIONS
    public void hardwareInit()
    {
        //RANDOM
        dashboard = FtcDashboard.getInstance();

        //PID
        ext = new PIDController(extP, extI, extD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //arm motors
        flipMotor = hardwareMap.get(DcMotorEx.class, "flip");
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //drive motors
        drive = new NewMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Mailbox.currentPose);

        //servos
        clawServo = hardwareMap.get(Servo.class, "claw");

        //gamepads
        currG1 = new Gamepad();
        oldG1 = new Gamepad();
        currG2 = new Gamepad();
        oldG2 = new Gamepad();


        //mailbox
        imu = hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
    public void movementInitI()
    {
        extTarget = 0;
        notNormalLimits = false;
    }
    public void movementInitII()
    {
        clawServo.setPosition(0.4);
        flpPosTarget = -200;
    }

    //BASICS
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardwareInit();

        movementInitI();
        clawIH = true;
        currentState = poseControlState.FREE;
        gameModeA = speedControlState.NORMAL;
        gameModeB = speedControlState.NORMAL;
        extLimit = 1600;
        flpLimit = 4300;

        waitForStart();

        movementInitII();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            mainLoop();
        }
    }
    public void mainLoop()
    {
        //UPDATES
        oldG1.copy(currG1);
        oldG2.copy(currG2);
        currG1.copy(gamepad1);
        currG2.copy(gamepad2);
        telemetry.update();
        drive.update();
        poseEstimate = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());

        //CONTROLS
        driverAControls();
        driverBControls();
        telemetry.addData("CURRENT POSITION STATE", currentState);
        stateCheck();

        //EXTENDER & FLIPPER
        extCONTROLLER();
        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
    }

    //DRIVING CONTROLS
    public void driverAControls()
    {
        //SPEED CHANGES | LEFT TRIGGER FAST | RIGHT TRIGGER SLOW
        if (gamepad1.right_trigger > 0) {
            gameModeA = speedControlState.PRECISION;
        }
        if (gamepad1.left_trigger > 0) {
            gameModeA = speedControlState.SUPERSPEED;
        }
        switch(gameModeA){
            case NORMAL:
                multiply = 1;
                speed = 3;
                break;
            case PRECISION:
                multiply = 0.5;
                speed = 3;
                break;
            case SUPERSPEED:
                multiply =0.7;
                speed = 1;
                break;
        }

        //FIELD CENTRIC RESET
        if(currG1.right_bumper && !currG1.right_bumper  )
        {
            drive.secretSetHeading();
        }

        //FIELD CENTRIC
        poseEstimate = drive.getPoseEstimate();
        Vector2d input = new Vector2d(
                -((gamepad1.left_stick_y)* multiply)/speed,
                -((gamepad1.left_stick_x)* multiply)/speed
        ).rotated(-poseEstimate.getHeading());
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        ((gamepad1.right_stick_x * multiply)/speed)
                )
        );
        drive.update();
    }
    public void driverBControls()
    {
        //region SPEED CONTROLS
        if ((currG2.right_trigger > 0.3) && !(oldG2.right_trigger>0.3) && (!notNormalLimits)) {
            gameModeB = speedControlState.PRECISION;
            notNormalLimits = true;
        }
        else if((currG2.right_trigger > 0.3) && !(oldG2.right_trigger>0.3))
        {
            notNormalLimits = false;
            gameModeB = speedControlState.NORMAL;
        }
        if ((currG2.left_trigger > 0.3) && !(oldG2.left_trigger>0.3) && (!notNormalLimits))  {
            gameModeB = speedControlState.SUPERSPEED;
            notNormalLimits = true;
        }
        else if((currG2.left_trigger > 0.3) && !(oldG2.left_trigger>0.3))
        {
            notNormalLimits = false;
            gameModeB = speedControlState.NORMAL;
        }

        double flpAddition = 70;
        double extAddition = 40;
        switch(gameModeB){
            case NORMAL:
                extLimit = 10000;
                flpLimit = 10000;
                break;
            case PRECISION:
                flpLimit = 8000;
                break;
            case SUPERSPEED:
                extLimit = 8000;
                break;
        }
        //endregion

        setStates();
        //region CLAW
        if (currG2.b && !oldG2.b)
        {
            if(clawIH)
            {
                clawServo.setPosition(0.4);
            }
            else {
                clawServo.setPosition(0);
            }
            clawIH = !clawIH;
        }
        //endregion

        //region EXTENDER
        if(gamepad2.dpad_up && extTarget<=extLimit)
        {
            telemetry.addLine("ext UP");
            currentState = poseControlState.FREE;
            if(extTarget+extAddition>=extLimit-extAddition)
            {
                extTarget+=Math.abs(Math.abs(extTarget)-extLimit);
            }
            else {
                extTarget+=extAddition;
            }
        }
        else if(gamepad2.dpad_down && extTarget>=250)
        {
            telemetry.addLine("ext DOWN");
            currentState = poseControlState.FREE;
            if(extTarget-extAddition<=250)
            {
                extTarget-=Math.abs(extTarget);
            }
            else {
                extTarget-=extAddition;
            }
        }
        //endregion

        //region FLIPPER
        if(gamepad2.dpad_right && flpPosTarget<=-150)
        {
            telemetry.addLine("flp DOWN");
            currentState = poseControlState.FREE;

            if(flpPosTarget+flpAddition>=-150)
            {
                flpPosTarget+=Math.abs(flpPosTarget+150);
            }
            else {
                flpPosTarget+=flpAddition;
            }
        }
        else if(gamepad2.dpad_left && flpPosTarget>=-flpLimit)
        {
            telemetry.addLine("flp UP");
            currentState = poseControlState.FREE;

            if(flpPosTarget-flpAddition<=-flpLimit)
            {
                flpPosTarget-=Math.abs(flpPosTarget+flpLimit);
            }
            else {
                flpPosTarget-=flpAddition;
            }
        }
        //endregion
    }
    public void setStates()
    {
        //region HIGH POSITION
        if(currentState!= poseControlState.HIGH && currG2.y && !oldG2.y)
        {
            telemetry.addLine("TO HIGH POSITION");
            currentState = poseControlState.HIGH;
            if(flpPosTarget>-2000) {
                extTarget = 1600;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    driverAControls();
                    flpPosTarget = -1650;
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
            else {

            }
        }
        //endregion

        //region LOW POSITION
        if(currentState!= poseControlState.LOW && currG2.left_bumper && !oldG2.left_bumper)
        {
            telemetry.addLine("TO LOW POSITION");
            currentState = poseControlState.LOW;
        }
        //endregion

        //region HOME POSITION
        if(currentState!= poseControlState.HOME && currG2.x && !oldG2.x)
        {
            telemetry.addLine("TO HOME POSITION");
            currentState = poseControlState.HOME;
        }
        //endregion

        //region PICKUP POSITION
        if(currentState!= poseControlState.PICKUP && currG2.a && !oldG2.a) {
            telemetry.addLine("TO PICKUP POSITION");
            currentState = poseControlState.PICKUP;
        }
        //endregion

        //region JERK
        if( flipMotor.getCurrentPosition()>-2000 && currG2.right_bumper && !oldG2.right_bumper) {

        }
        //endregion
    }

    //CONTROLLERS
    public void extCONTROLLER()
    {
        ext.setPID(extP, extI, extD);
        int extPose = armMotor.getCurrentPosition();
        double extPwr = ext.calculate(extPose, extTarget) + (Math.cos(Math.toRadians(extTarget/ticksPerDegree)) * extF);
        armMotor.setPower(extPwr *(1/3.0));

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

    //TELEMETRY
    public void stateCheck()
    {
        if(clawIH)
        {
            telemetry.addLine("CLAW CLOSE");
        }
        else{
            telemetry.addLine("CLAW OPEN");
        }

        telemetry.addData("GAMEMODEB", gameModeB);
        telemetry.addData("flpVelocity ", flipMotor.getVelocity());
        telemetry.addData("flpPosition ", flipMotor.getCurrentPosition());
        telemetry.addData("flpTarget", flpPosTarget);

        telemetry.addData("ext TARGET - ", extTarget);
        telemetry.addData("claw - ", clawServo.getPosition());
    }
}

//0.3317 0.2789
