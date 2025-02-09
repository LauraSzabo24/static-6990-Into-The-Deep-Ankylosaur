package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import java.util.ArrayList;
import android.graphics.Color;


import Autonomous.Mailbox;

@TeleOp
@Config
public class SubTests extends LinearOpMode {

    //region DRIVER A MATERIAL
    NewMecanumDrive drive;
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

    //region GAMEPADS
    Gamepad currG1;
    Gamepad oldG1;
    Gamepad currG2;
    Gamepad oldG2;
    //endregion

    //region SENSORS
    ColorSensor colorDetector;
    ElapsedTime colorTimer = new ElapsedTime();
    float hsvValues[];
    private enum color{
        RED,
        YELLOW,
        BLUE,
        GRAY
    }
    double red, green, blue;
    double perRed, perGreen, perBlue;
    ArrayList<color> detections;
    ArrayList<Integer>  goodStreakLengths;
    ArrayList<Double>  streakPosStart, streakPosEnd;

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

    //INITIALIZATIONS
    public void hardwareInit()
    {
        //RANDOM
        colorDetector = hardwareMap.get(RevColorSensorV3.class, "color");
        detections = new ArrayList<color>();
        goodStreakLengths = new ArrayList<>();
        streakPosStart = new ArrayList<>();
        streakPosEnd = new ArrayList<>();

        //PID
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //arm motors
        /*flipMotor = hardwareMap.get(DcMotorEx.class, "FLIP");
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
        extRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        //drive motors
        /*drive = new NewMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.reverseMotors();
        drive.setPoseEstimate(Mailbox.currentPose);*/

        //servos
        smallWrist = hardwareMap.get(Servo.class, "SW");
       // bigWristL = hardwareMap.get(Servo.class, "BWL");
        //bigWristR = hardwareMap.get(Servo.class, "BWR");
        spin = hardwareMap.get(Servo.class, "SPIN");
        claw = hardwareMap.get(Servo.class, "CLAW");

        //gamepads
        currG1 = new Gamepad();
        oldG1 = new Gamepad();
        currG2 = new Gamepad();
        oldG2 = new Gamepad();
    }

    //BASICS
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardwareInit();
        spin.setPosition(0.1528);
        smallWrist.setPosition(0.98);
        claw.setPosition(1);
        colorDetector.enableLed(true);

        clawIH = true;
        controlState = poseControlState.FREE;
        gameModeA = speedControlState.NORMAL;
        gameModeB = speedControlState.NORMAL;
        hangState = hangControlState.PHASEONE;
        editMode = true;
        waitForStart();

        //movementInitII();
        double cumutime = 0;
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

        //CONTROLS
        driverBControls();
        telemetry.addData("CURRENT POSITION STATE", controlState);
        printColors();
    }

    //DRIVING CONTROLS
    public void driverBControls()
    {
        //region TRANSITION
        if(gamepad2.right_bumper)
        {
            editMode = false;
            gameModeB = speedControlState.NORMAL;
            notNormalLimits = false;
            gamepad2.setLedColor((38/255.0), (255/255.0), 0, Gamepad.LED_DURATION_CONTINUOUS);

        }
        else {
            editMode = true;
            gamepad2.setLedColor((255/255.0), 0, (125/255.0), Gamepad.LED_DURATION_CONTINUOUS);
        }
        //endregion

        if(editMode)
        {
            //region SPEED CONTROLS | LEFT TRIGGER FAST | RIGHT TRIGGER SLOW
            int extAmount = 40;
            double smallAmount = 0.01;
            double bigAmount = 0.03;
            gameModeB = speedControlState.NORMAL;
            if (gamepad2.right_trigger > 0.3) {
                gameModeB = speedControlState.PRECISION;
            }
            if (gamepad2.left_trigger > 0.3) {
                gameModeB = speedControlState.SUPERSPEED;
            }
            switch(gameModeB){
                case NORMAL:
                    break;
                case PRECISION:
                    extAmount = 20;
                    smallAmount = 0.004;
                    bigAmount = 0.01;
                    break;
                case SUPERSPEED:
                    extAmount = 80;
                    smallAmount = 0.03;
                    bigAmount = 0.06;
                    break;
            }
            //endregion

            //region SMALL WRIST
            if(gamepad2.left_stick_x>0 && smallWrist.getPosition()<(0.96-smallAmount))
            {
                telemetry.addLine("WRIST MOVEMENT");
                smallWrist.setPosition(smallWrist.getPosition() + smallAmount);
                controlState = poseControlState.FREE;
            }
            else if(gamepad2.left_stick_x<0 && smallWrist.getPosition()>=0.1367-smallAmount)
            {
                telemetry.addLine("WRIST MOVEMENT");
                smallWrist.setPosition(smallWrist.getPosition() - smallAmount);
                controlState = poseControlState.FREE;
            }
            if(currG2.left_stick_button && !oldG2.left_stick_button)
            {
                telemetry.addLine("WRIST MOVEMENT");
                controlState = poseControlState.FREE;
                smallWristPos ++;
                if(smallWristPos>2)
                {
                    smallWristPos = 0;
                }
                switch(smallWristPos)
                {
                    case 0:
                        smallWrist.setPosition(0.1367);
                        break;
                    case 1:
                        smallWrist.setPosition(0.5667);
                        break;
                    case 2:
                        smallWrist.setPosition(0.96);
                        break;
                }
            }
            //endregion

            //region SPINNER
            if(gamepad2.right_stick_x>0 || gamepad2.right_stick_x<0)
            {
                telemetry.addLine("SPINNER MOVEMENT");
                spin.setPosition(spin.getPosition() + (gamepad2.right_stick_x * 0.05));
                if(controlState != poseControlState.VARIPICKUP)
                {
                    controlState = poseControlState.FREE;
                }            }
            if(currG2.right_stick_button && !oldG2.right_stick_button)
            {
                spinnerPos ++;
                if(spinnerPos>3)
                {
                    spinnerPos = 0;
                }
                switch(spinnerPos)
                {
                    case 0:
                        spin.setPosition(0.1528);
                        break;
                    case 1:
                        spin.setPosition(0.4239);
                        break;
                    case 2:
                        spin.setPosition(0.7111);
                        break;
                    case 3:
                        spin.setPosition(1);
                        break;
                }
            }
            //endregion

            /*
            //region EXTENDER
            if(gamepad2.dpad_up && extTarget<=1520)
            {
                telemetry.addLine("ext UP");
                if(controlState != poseControlState.VARIPICKUP)
                {
                    controlState = poseControlState.FREE;
                }
                if(flpPosTarget<200) //200
                {
                    if (extTarget + extAmount >= 1520 - extAmount) {
                        extTarget += Math.abs(Math.abs(extTarget) - 1520);
                    } else {
                        extTarget += extAmount;
                    }
                }
                else if(flpPosTarget>=1600)
                {
                    if(extTarget<=700) {
                        if (extTarget + extAmount >= 700 - extAmount) {
                            extTarget += Math.abs(Math.abs(extTarget) - 700);
                        } else {
                            extTarget += extAmount;
                        }
                    }
                }
            }
            else if(gamepad2.dpad_down && extTarget>=0)
            {
                telemetry.addLine("ext DOWN");
                if(controlState != poseControlState.VARIPICKUP)
                {
                    controlState = poseControlState.FREE;
                }
                if(flpPosTarget<200) { //200
                    if (extTarget - extAmount <= 0) {
                        extTarget -= Math.abs(extTarget);
                    } else {
                        extTarget -= extAmount;
                    }
                }
                else if(flpPosTarget>=1600)
                {
                    if (extTarget - extAmount <= extAmount) {
                        extTarget -= Math.abs(extTarget-extAmount);
                    } else {
                        extTarget -= extAmount;
                    }
                }
            }
            //endregion

            //region VARIABLE PICKUP
            if(controlState == poseControlState.VARIPICKUP)
            {
                extPercentage = (extTarget-40.0)/700;
                spin.setPosition(0.7106);
                smallWrist.setPosition(0.1739 + ((0.1856-0.1739)*extPercentage));
                bigWristL.setPosition(0.4289 + ((0.4089-0.4289)*extPercentage));
                bigWristR.setPosition(0.57 + ((0.59-0.57)*extPercentage));

                //EXT          40         700
                //smallWrist - 0.1739 - 0.1856
                //bigWristL - 0.4289 - 0.4089
                //bigWristR - 0.57- 0.59
            }
            //endregion

            //region FLIPPER
            if(gamepad2.dpad_left && flpPosTarget>=flpLowLimit)
            {
                telemetry.addLine("flp UP");
                controlState = poseControlState.FREE;
                if(flpPosTarget-20<=0)
                {
                    flpPosTarget-=Math.abs(flpPosTarget);
                }
                else {
                    flpPosTarget-=20;
                }
            }
            else if(gamepad2.dpad_right && flpPosTarget<flpHighLimit && !(extTarget>500 && flpPosTarget<2500 && flpHighLimit==3500))
            {
                telemetry.addLine("flp DOWN");
                controlState = poseControlState.FREE;
                if(flpPosTarget+20>=flpHighLimit)
                {
                    flpPosTarget+=Math.abs(flpHighLimit-flpPosTarget);
                }
                else {
                    flpPosTarget+=20;
                }
            }

            //COMBO MOVEMENT EXTENSION
           /* if(extTarget>=200)
            {
                double flpPercentage = (flpPosTarget/1620.0);
                extTarget -= extTarget*flpPercentage;
            }*/
            //endregion*/

            //region CLAW
            if (currG2.touchpad && !oldG2.touchpad)
            {
                if(clawIH)
                {
                    claw.setPosition(0.9);
                }
                else {
                    claw.setPosition(0.3);
                }
                clawIH = !clawIH;
                if(controlState == poseControlState.VARIPICKUP || controlState == poseControlState.PICKUP)
                {
                    variablePickup = !variablePickup;
                }
            }
            //endregion

            //region BLOCK PICKUP
            if(currG2.left_bumper && !oldG2.left_bumper)
            {
                pickupBlock();
            }
            //endregion
        }
    }
    public void pickupBlock()
    {
        //region COLORS
        detections.removeAll(detections);
        double[] redHigh = new double[]{145,65,30};
        double[] redLow = new double[]{159,72,37};
        double[] yellowHigh = new double[]{115,122,28};
        double[] yellowLow = new double[]{103,114,25};
        double[] blueHigh = new double[]{37,64,160};
        double[] blueLow = new double[]{32,62,153};
        //endregion

        //region SPIN AROUND
        spin.setPosition(0);
        sleep(2000);
        for(int i=0; i<24; i++)
        {
            spin.setPosition(spin.getPosition() + 0.0416667);
            colorTimer.reset();
            sleep(10);
            double white = colorDetector.red()+colorDetector.green()+colorDetector.blue();
            perRed = colorDetector.red()/white;
            perGreen = colorDetector.green()/white;
            perBlue = colorDetector.blue()/white;
            telemetry.addLine("PEACH");
            telemetry.addData("% RED", perRed);
            telemetry.addData("% GREEN", perGreen);
            telemetry.addData("% BLUE", perBlue);
            telemetry.update();
            sleep(10);

            if(perRed>0.52)
            {
                detections.add(color.RED);
            }
            else if(perBlue<0.14 && ((perRed-perGreen)<=0.05))
            {
                detections.add(color.YELLOW);
            }
            else if(perBlue>0.52)
            {
                detections.add(color.BLUE);
            }
            else{
                detections.add(color.GRAY);
            }
        }
        //endregion

        //region FILL HOLES
        if((!detections.get(0).equals(detections.get(23))) && (!detections.get(0).equals(detections.get(1))) && (detections.get(23).equals(detections.get(1))))
        {
            detections.set(0,detections.get(1));
        }
        if((!detections.get(23).equals(detections.get(22))) && (!detections.get(23).equals(detections.get(0))) && (detections.get(22).equals(detections.get(0))))
        {
            detections.set(23,detections.get(0));
        }
        for(int i=1; i<23; i++)
        {
            if((!detections.get(i).equals(detections.get(i-1))) && (!detections.get(i).equals(detections.get(i+1))) && (detections.get(i-1).equals(detections.get(i+1))))
            {
                detections.set(i,detections.get(i+1));
            }
        }
        //endregion

        //region FIND RANGES
        int streak = 0;
        goodStreakLengths.removeAll(goodStreakLengths);
        streakPosStart.removeAll(streakPosStart);
        streakPosEnd.removeAll(streakPosEnd);
        double currPos = 0.0416667*2;
        for(int i=0; i<24; i++)
        {
            currPos += 0.0416667;
            if(detections.get(i).equals(color.YELLOW))
            {
                streak++;
                if(streak==1)
                {
                    streakPosStart.add(currPos);
                }
            }
            else{
                if(streak>2)
                {
                    goodStreakLengths.add(streak);
                    streakPosEnd.add(currPos);
                }
                streak = 0;
            }
        }
        if(streak>2)
        {
            goodStreakLengths.add(streak);
            streakPosEnd.add(currPos);
        }
        //endregion

        //region FIGURE IT OUT
        double clawPos = 0;
        switch(goodStreakLengths.size()){
            case 0:
                //try again
                break;
            case 1:
                clawPos = streakPosStart.get(0);
                //clawPos = streakPosStart.get(0)+ (Math.abs(streakPosEnd.get(0)-streakPosStart.get(0))/2);
                break;
            default:
                clawPos = streakPosStart.get(0);
                break;
        }
        spin.setPosition(clawPos);
        sleep(800);
        claw.setPosition(0.7);
        //endregion

        //region DOUBLE CHECK

        //endregion
    }
    public void printColors()
    {
        String detects = "";
        String deetects = "";
        for(int i=0; i<23; i++)
        {
            if(detections.size()>i)
            {
                detects+= detections.get(i) + " ";
            }
        }
        telemetry.addLine("DETECTIONS " + detects);
        telemetry.addLine("GOOD STREAK COUNT " + goodStreakLengths.size());

        double white = colorDetector.red()+colorDetector.green()+colorDetector.blue();
        red = ((colorDetector.red())/white)*255;
        green = (colorDetector.green()/white)*255;
        blue = (colorDetector.blue()/white)*255;
        telemetry.addData("RED", red);
        telemetry.addData("GREEN", green);
        telemetry.addData("BLUE", blue);
        telemetry.addData("WHITE", white);


    }
}