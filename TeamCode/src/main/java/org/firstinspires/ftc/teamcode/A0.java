package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="#T>Auto0", group="Linear Opmode")
//@Disabled
public class A0 extends LinearOpMode {

    // Declare Hardware, timing, etc
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveFL = null;
    private DcMotor driveFR = null;
    private DcMotor driveRL = null;
    private DcMotor driveRR = null;
    private Servo mechGrab  = null;
    private ColorSensor sColor = null;
    private DistanceSensor sDist = null;

    @Override
    public void runOpMode() throws InterruptedException
    {

        //Setup Hardware
        driveFL = hardwareMap.get(DcMotor.class, "FL");
        driveFR = hardwareMap.get(DcMotor.class, "FR");
        driveRL = hardwareMap.get(DcMotor.class, "RL");
        driveRR = hardwareMap.get(DcMotor.class, "RR");
        mechGrab = hardwareMap.get(Servo.class, "GR");

        sColor = hardwareMap.get(ColorSensor.class, "CLR");
        sDist = hardwareMap.get(DistanceSensor.class, "CLR");

        driveFL.setDirection(DcMotor.Direction.FORWARD);
        driveFR.setDirection(DcMotor.Direction.REVERSE);
        driveRL.setDirection(DcMotor.Direction.FORWARD);
        driveRR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //INPUT
        double vX=0, vY=0, vR=0;

        double LocalX = 0, LocalY = 0; //robot speed in local coordinates
        double X=0, Y=0, A=0;
        //OUTPUT
        double  power_FL, power_FR,

                power_RL, power_RR;

        double  cR=0,
                cG=0,
                cB=0;


        // run until the end of the match (driver presses STOP)

        Drive(0,.5,0,700);

        while(sDist.getDistance(DistanceUnit.CM)>=3){
            Drive(0,.45,0,100);
        }



            cR = sColor.red();
            cG = sColor.green();
            cB = sColor.blue();

            // Send calculated power to wheels


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("IN", "Y (%.2f), X (%.2f)", vY, vX);
            telemetry.addData("Color", "R%.2f\nG%.2f\nB%.2f", cR, cG, cB);
            //telemetry.addData("Motors", "FL (%.2f), FR (%.2f), RL (%.2f), RR (%.2f)", power_FL, power_FR, power_RL, power_RR);

            telemetry.update();


    }

    public void Drive(double X,double Y,double R, long time) throws InterruptedException
    {
        driveFL.setPower ( X +Y -R);
        driveFR.setPower (-X +Y +R);
        driveRL.setPower (-X +Y -R);
        driveRR.setPower ( X +Y +R);
        Thread.sleep(time);
    }

    public void Stop() throws InterruptedException
    {
        Drive (0,0,0,0);
    }
}