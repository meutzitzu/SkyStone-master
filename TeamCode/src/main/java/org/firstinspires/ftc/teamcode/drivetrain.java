package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    @Override
    public void runOpMode() {

        //Setup Hardware
        driveFL = hardwareMap.get(DcMotor.class, "FL");
        driveFR = hardwareMap.get(DcMotor.class, "FR");
        driveRL = hardwareMap.get(DcMotor.class, "RL");
        driveRR = hardwareMap.get(DcMotor.class, "RR");

        driveFL.setDirection(DcMotor.Direction.FORWARD);
        driveFR.setDirection(DcMotor.Direction.REVERSE);
        driveRL.setDirection(DcMotor.Direction.FORWARD);
        driveRR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //INPUT
        double vX, vY, vR;     //robot speed as commanded by driver (can be in either LOCAL or GLOBAL coordinates, depending on the MODE)

        //CTRL
        int MODE = 0;
                            /* MODE 0 means simple "first person" controls, no trig tricks needed,
                             MODE 1 means controlling the robot in GLOBAL coordinates
                             (left's always left, right's always right, regardless of robot orientation)(some trig required)
                             see MODE STATE MACHINE */


        double LocalX = 0, LocalY = 0; //robot speed in local coordinates
        double X=0, Y=0, A=0;
        //OUTPUT
        double  power_FL, power_FR,

                power_RL, power_RR;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //READ INPUTS
            vY = -gamepad1.left_stick_y;
            vX = gamepad1.left_stick_x;
            vR = gamepad1.right_trigger - gamepad1.left_trigger;

            if (gamepad1.x&&gamepad1.y) {
                if (gamepad1.dpad_right) {
                    MODE = 0;
                } else if (gamepad1.dpad_left){
                    MODE = 1;
                }
            }


            /*
                UPDATE X,Y,A from sensors
             */


            switch (MODE) {
                case 0: {
                    LocalY = vY;
                    LocalX = vX;
                    break;
                }

                case 1: {
                    A = Math.PI / 2;
                    LocalY = vX * Math.sin(A) + vY * Math.cos(A);
                    LocalY = vX * Math.cos(A) - vY * Math.sin(A);
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
            telemetry.addData("IN", "Y (%.2f), X (%.2f)", vY, vX);

            telemetry.addData("Motors", "FL (%.2f), FR (%.2f), RL (%.2f), RR (%.2f)", power_FL, power_FR, power_RL, power_RR);

            telemetry.update();

        }
    }
}