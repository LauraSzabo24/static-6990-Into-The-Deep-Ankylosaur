/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/
package org.firstinspires.ftc.robotcontroller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Config
@TeleOp(name = "Concept Datalogger v01", group = "Datalogging")
public class ConceptDatalogger extends LinearOpMode
{
    Datalog datalog;
    BNO055IMU imu;
    public static String numLog = "08";

    //public static double power = 0.05;
    public static double velocity = 0.05;

    public boolean beenHere = false;

    //region FLIPPER CONTROLLER
    //POSITION
    ElapsedTime timer = new ElapsedTime();
    private double flpPosError = 0;
    private double flpPosISum = 0;

    public  double flpPP = 0.1, flpPI = 0, flpPD = 0;
    public  int flpPosTarget = 0;

    //VELOCITY
    private double flpVeloError = 0;
    private double flpVeloISum = 0;
    public int flpVeloTarget = 0;
    public int testTarget = 1200;
    public double flpVP = 0.0002, flpVI = 0.004, flpVD = 0.0000001;  //RISING
    //endregion

    //region DRIVER B MATERIAL
    private Servo wristServo, spinnerServo, clawServo;
    DcMotorEx flipMotor, armMotor;
    boolean clawIH;
    boolean pickupTwo = false;
    ElapsedTime jerkTimer = new ElapsedTime();
    boolean jerked = false;
    //endregion

    //region GAMEPADS
    Gamepad currG1;
    Gamepad oldG1;
    Gamepad currG2;
    Gamepad oldG2;

    //endregion

    //region CONTROL STATE
    private enum controlStateB
    {
        FREE,
        PICKUP,
        LOW,
        HIGH
    }
    private enum controlStateA
    {
        UNLIMITED,
        LIMITED,
        HANG
    }
    controlStateB gameModeB;
    controlStateA gameModeA;
    //endregion

    @Override
    public void runOpMode() throws InterruptedException
    {
        //arm motors
        flipMotor = hardwareMap.get(DcMotorEx.class, "flip");
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the datalog
        datalog = new Datalog("datalog_" + numLog);

        datalog.opModeStatus.set("INIT");
        datalog.writeLine();

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        datalog.opModeStatus.set("RUNNING");

        double cumutime = 0;
        if (isStopRequested()) return;

        for (int i = 0; opModeIsActive(); i++) {

            /*double time = timer.seconds();
            timer.reset();
            cumutime+=time;

            int targetPositionYay = 0;
            if(gamepad1.a)
            {
                targetPositionYay = -testTarget;
            }
            if(gamepad1.b)
            {
                targetPositionYay = testTarget;
            }
            //flpCONTROLLER(targetPositionYay, flipMotor.getCurrentPosition());
            double needPower = flpVelocityCONTROLLER(targetPositionYay,flipMotor.getVelocity(), time);
            flipMotor.setPower(needPower);
            //telemetry.addData("targets", targetPositionYay);
            telemetry.addData("power",needPower);
            telemetry.addData("time",cumutime);
            telemetry.update();

            if((flipMotor.getCurrentPosition()>-900 && flipMotor.getVelocity()<0) ||(flipMotor.getCurrentPosition()<-900 && flipMotor.getVelocity()>0))
            {
                datalog.opModeStatus.set("RISING");
            }
            else {
            }*/

            if (flipMotor.getCurrentPosition()<=-4000)
            {
                beenHere = true;
                //flipMotor.setPower(power);
                flipMotor.setVelocity(velocity);
                datalog.opModeStatus.set("BACK");
            }
            else if (!beenHere) {
                //flipMotor.setPower(-power);
                flipMotor.setVelocity(-velocity);
                datalog.opModeStatus.set("FORWARD");
            }

            if(flipMotor.getCurrentPosition()>=-200 && beenHere)
            {
                //flipMotor.setPower(0);
                flipMotor.setVelocity(0);
            }

            datalog.loopCounter.set(i);

            //datalog.time.set(cumutime);
            datalog.velocity.set(flipMotor.getVelocity());
            datalog.power.set(flipMotor.getPower());
            //datalog.power.set(needPower);
            datalog.position.set(flipMotor.getCurrentPosition());

            datalog.writeLine();
        }
    }
    public double flpVelocityCONTROLLER(double target, double state, double time) //in with the velocity -> out with the power
    {
        /*if(target<=-300)
        {
            target = -300;
        }*/
        double currError = (target - state);
        flpVeloISum += currError * time;
        double deriv = (currError - flpVeloError)/time;
        flpVeloError = currError;

        telemetry.addData("velocity",flipMotor.getVelocity());
        telemetry.addData("position",flipMotor.getCurrentPosition());

        if((flipMotor.getCurrentPosition()>-900 && target>0) || (flipMotor.getCurrentPosition()<-900 && target<0)) //IS FALLING
        {
            telemetry.addLine("FALLING");
            telemetry.addData("ERROR", currError);
            return (flpVP * currError) + (flpVI *flpVeloISum) + (flpVD * deriv);
        }
        else //IS RISING
        {
            telemetry.addLine("RISING");
            return (flpVP * currError) + (flpVI *flpVeloISum) + (flpVD * deriv);
        }
    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("STATE");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("COUNT");
        public Datalogger.GenericField power         = new Datalogger.GenericField("POWER");
        public Datalogger.GenericField velocity        = new Datalogger.GenericField("VELOCITY");
        public Datalogger.GenericField time        = new Datalogger.GenericField("TIME");
        public Datalogger.GenericField position        = new Datalogger.GenericField("POSITION");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            loopCounter,
                            time,
                            velocity,
                            power,
                            position
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}
