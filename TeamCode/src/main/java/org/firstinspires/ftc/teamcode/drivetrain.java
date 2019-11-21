package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
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


@TeleOp(name="#T>drivetrain", group="Linear Opmode")
//@Disabled
public class drivetrain extends LinearOpMode {

    // Declare Hardware, timing, etc
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveFL = null;
    private DcMotor driveFR = null;
    private DcMotor driveRL = null;
    private DcMotor driveRR = null;
    private Servo  mechGrab = null;
    private ColorSensor sColor = null;
    private DistanceSensor sDist = null;
    private ModernRoboticsI2cCompassSensor sCompass = null;

    @Override
    public void runOpMode() {

        //Setup Hardware
        driveFL = hardwareMap.get(DcMotor.class, "FL");
        driveFR = hardwareMap.get(DcMotor.class, "FR");
        driveRL = hardwareMap.get(DcMotor.class, "RL");
        driveRR = hardwareMap.get(DcMotor.class, "RR");
        mechGrab = hardwareMap.get(Servo.class, "GR");
        sColor = hardwareMap.get(ColorSensor.class, "CLR");
        sDist = hardwareMap.get(DistanceSensor.class, "CLR");
        sCompass = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "HDG");


        driveFL.setDirection(DcMotor.Direction.FORWARD);
        driveFR.setDirection(DcMotor.Direction.REVERSE);
        driveRL.setDirection(DcMotor.Direction.FORWARD);
        driveRR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //INPUT
        double vX, vY, vR;     //robot speed as commanded by driver (can be in either LOCAL or GLOBAL coordinates, depending on the MODE)

        //SENSOR INPUT
        double  cR=0,
                cG=0,
                cB=0;
        //CTRL
        int MODE = 0;
        double delay = 0 ;          //stiu ca e cancer da atat s-a putut
                            /* MODE 0 means simple "first person" controls, no trig tricks needed,
                             MODE 1 means controlling the robot in GLOBAL coordinates
                             (left's always left, right's always right, regardless of robot orientation)(some trig required)
                             see MODE STATE MACHINE */

        int GEAR = 1;
        double Gear_shift = 0;
        double LocalX = 0, LocalY = 0; //robot speed in local coordinates
        double X=0, Y=0, A=0;
        double A0 =0;
        //OUTPUT
        double  power_FL, power_FR,

                power_RL, power_RR;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //READ INPUTS
            vY = -gamepad1.left_stick_y;
            vX = gamepad1.left_stick_x;
            vR = gamepad1.right_stick_x;

            A = A0 + Math.toRadians(sCompass.getDirection());

            cR = sColor.red();
            cG = sColor.green();
            cB = sColor.blue();


            if (runtime.seconds()-delay > .3){
                if(gamepad1.x){
                    GEAR ++;

                } else if( GEAR > 4) {
                    GEAR = 1;
                }
                if(gamepad1.y) {
                    if(MODE == 0){
                        MODE = 1;
                    }
                    else if(MODE ==1 ){
                        MODE = 0;
                    }
                }
                if(gamepad1.x && gamepad1.b){
                    A0 = A;
                }
                delay = runtime.seconds();
            }

            if(gamepad1.dpad_down){
                mechGrab.setPosition(0);
            } else if (gamepad1.dpad_up) {
                mechGrab.setPosition(1);
            }






            switch (GEAR){
                case 1 : {
                    Gear_shift = .25;
                    break;
                }
                case 2 : {
                    Gear_shift = .4;
                    break;
                }
                case 3 : {
                    Gear_shift = .65;
                    break;
                }
                case 4 : {
                    Gear_shift = 1;
                    break;
                }
            }

            //Apply GEAR
            vX *= Gear_shift;
            vY *= Gear_shift;
            vR *= Gear_shift;

            switch (MODE) {
                case 0: {
                    LocalY = vY;
                    LocalX = vX;
                    break;
                }

                case 1: {
                    LocalX = vX * Math.cos(A) - vY * Math.sin(A);
                    LocalY = vY * Math.sin(A) + vX * Math.cos(A);
                    break;
                }
            }

            // O D O M E T R Y goes HERE !!

            power_FL = Range.clip(+LocalX + LocalY + vR, -1.0, 1.0);
            power_FR = Range.clip(-LocalX + LocalY - vR, -1.0, 1.0);
            power_RL = Range.clip(-LocalX + LocalY + vR, -1.0, 1.0);
            power_RR = Range.clip(+LocalX + LocalY - vR, -1.0, 1.0);


            // Send calculated power to wheels
            driveFL.setPower(power_FL);
            driveFR.setPower(power_FR);
            driveRL.setPower(power_RL);
            driveRR.setPower(power_RR);
            

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("MODE", ": (%d)", MODE);
            telemetry.addData("GEAR", ": %.2f", Gear_shift);
            telemetry.addData("HDG","%.2f",Math.toDegrees(A));
            telemetry.addData("Color", "R%.2f\nG%.2f\nB%.2f", cR, cG, cB);
            //telemetry.addData("IN", "Y (%.2f), X (%.2f)", vY, vX);

            telemetry.addData("Motors", "FL (%.2f), FR (%.2f), RL (%.2f), RR (%.2f)", power_FL, power_FR, power_RL, power_RR);

            telemetry.update();

        }
    }
}