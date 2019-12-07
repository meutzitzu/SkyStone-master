package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareDrivetrain;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@TeleOp(name="#T>PIDdrivetrain", group="Linear Opmode")
//@Disabled
public class PIDdrivetrain extends LinearOpMode {

    // Declare Hardware, timing, etc
    HardwareDrivetrain drive   = new HardwareDrivetrain();
    private ElapsedTime runtime = new ElapsedTime();
    private Servo  mechGrab = null;
    private ColorSensor sColor = null;
    private DistanceSensor sDist = null;
    private ModernRoboticsI2cCompassSensor sCompass = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_CM       = 4.0 ;
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
                                                        (WHEEL_DIAMETER_CM * Math.PI);

    @Override
    public void runOpMode() {

        //Setup Hardware
        drive.init(hardwareMap);
        drive.FL = hardwareMap.get(DcMotor.class, "FL");
        drive.FR = hardwareMap.get(DcMotor.class, "FR");
        drive.RL = hardwareMap.get(DcMotor.class, "RL");
        drive.RR = hardwareMap.get(DcMotor.class, "RR");
        mechGrab = hardwareMap.get(Servo.class, "GR");
        sColor = hardwareMap.get(ColorSensor.class, "CLR");
        sDist = hardwareMap.get(DistanceSensor.class, "CLR");
        sCompass = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "HDG");


        drive.FL.setDirection(DcMotor.Direction.FORWARD);
        drive.FR.setDirection(DcMotor.Direction.REVERSE);
        drive.RL.setDirection(DcMotor.Direction.FORWARD);
        drive.RR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //INPUT
        double vX, vY, vR;     //robot speed as commanded by driver (can be in either LOCAL or GLOBAL coordinates, depending on the MODE)

        //SENSOR INPUT
        double  cR=0,
                cG=0,
                cB=0;
        double X=0, Y=0, A=0; //robot coordinates (for odometry) {note that we can not use these yet because we have no odometry sensors}

        //CTRL
        double LocalX = 0, LocalY = 0; //robot speed in local coordinates
        double delay = 0 ;          //stiu ca e cancer da atat s-a putut
        int MODE = 0;
        double A0 =0;
                            /* MODE 0 means simple "first person" controls, no trig tricks needed,
                             MODE 1 means controlling the robot in GLOBAL coordinates
                             (left's always left, right's always right, regardless of robot orientation)(some trig required)
                             see MODE STATE MACHINE */

        int GEAR = 1;
        double Gear_shift = 0;
        //OUTPUT
        double  power_FL, power_FR,

                power_RL, power_RR;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //READ INPUTS
            vY = -gamepad1.left_stick_y;
            vX = gamepad1.left_stick_x;
            vR = gamepad1.right_stick_x;

            A = A0 + Math.toRadians((double)sCompass.getDirection());

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

            if(gamepad1.dpad_up){
                mechGrab.setPosition(0);
            } else if (gamepad1.dpad_down) {
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
                    LocalY = vX * Math.sin(A) + vY * Math.cos(A);
                    break;
                }
            }

            // O D O M E T R Y goes HERE !!

            power_FL = Range.clip(+LocalX + LocalY + vR, -1.0, 1.0);
            power_FR = Range.clip(-LocalX + LocalY - vR, -1.0, 1.0);
            power_RL = Range.clip(-LocalX + LocalY + vR, -1.0, 1.0);
            power_RR = Range.clip(+LocalX + LocalY - vR, -1.0, 1.0);


            // Send calculated power to wheels
            drive.FL.setPower(power_FL);
            drive.FR.setPower(power_FR);
            drive.RL.setPower(power_RL);
            drive.RR.setPower(power_RR);
            

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("MODE", ": (%d)", MODE);
            telemetry.addData("GEAR", ": %.2f", Gear_shift);
            telemetry.addData("IN","X%.2f Y%.2f",LocalX,LocalY);
            telemetry.addData("HDG","%.2f",A);
            telemetry.addData("Color", "\nR%.2f\nG%.2f\nB%.2f", cR, cG, cB);
            telemetry.addData("CntFL", " [ %d ]  ", drive.FL.getCurrentPosition());

            telemetry.addData("Motors", "FL (%.2f), FR (%.2f), RL (%.2f), RR (%.2f)", power_FL, power_FR, power_RL, power_RR);

            telemetry.update();

        }
    }

    public void drivePID(double speed,
                         double X,
                         double Y,
                         double rot,
                         double timeout){
        int TargetFL;
        int TargetFR;
        int TargetRL;
        int TargetRR;

        if(opModeIsActive()){
            TargetFL = drive.FL.getCurrentPosition() + (int)(Y * COUNTS_PER_CM);
            TargetFR = drive.FR.getCurrentPosition() + (int)(Y * COUNTS_PER_CM);
            TargetRL = drive.RL.getCurrentPosition() + (int)(Y * COUNTS_PER_CM);
            TargetRR = drive.RR.getCurrentPosition() + (int)(Y * COUNTS_PER_CM);

            drive.FL.setTargetPosition(TargetFL);
            drive.FR.setTargetPosition(TargetFR);
            drive.RL.setTargetPosition(TargetRL);
            drive.RR.setTargetPosition(TargetRR);


        }

    }
}