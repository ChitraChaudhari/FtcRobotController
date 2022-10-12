package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.FFRobot;

@TeleOp
public class TeleOpClaw extends OpMode {
    FFRobot robot = new FFRobot();

    double drive = 0.0;
    double strafe = 0.0;
    double turn = 0.0;

    //private ElapsedTime runtime = new ElapsedTime();

    Servo claw;

    @Override
    public void init(){

        robot.init(hardwareMap);
        claw = hardwareMap.get(Servo.class, "ClawServo");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Arm Motor Encoders");
        telemetry.update();

        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        /********************************
         * Mecanum Drive *
         *******************************/
        //controlling mecanum drive with gamepad1 left and right joystick
        drive = gamepad1.left_stick_y;          //drive front or back
        strafe = gamepad1.left_stick_x * 1.1;    //drive right or left
        //multiplying the left X value by 1.1 to
        // counteract imperfect strafing.
        turn = gamepad1.right_stick_x;           //turn clockwise or anticlockwise

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        //set the motors power
        robot.frontLeftMotor.setPower((drive + strafe + turn) / denominator);
        robot.backLeftMotor.setPower((drive - strafe + turn) / denominator);
        robot.frontRightMotor.setPower((drive - strafe - turn) / denominator);
        robot.backRightMotor.setPower((drive + strafe - turn) / denominator);

        /****************************************
         *  Robot Arm Controls *
         ***************************************/
        //Gamepad2 buttons moves the robot arm to different positions
        if (gamepad2.y) {
            robot.driveArmToPosition(0.75, 1275);
        }
        if (gamepad2.a) {
            robot.driveArmToPosition(0.75, 300);//
        }
        if (gamepad2.b) {
            robot.driveArmToPosition(0.75, 650);
        }
        if (gamepad2.x) {
            robot.driveArmToPosition(0.5, -300);
            //robot.ArmMotor.setPower(0.0);
        }
        if (gamepad2.right_bumper) {
            robot.ArmMotor.setPower(0.0);
        }
        //robot.ArmMotor.setPower(gamepad2.left_stick_y);

        /********************************************
         *  Turn table Mechanism Controls *
         *******************************************/

        robot.TurnTable.setPower(gamepad2.left_stick_x);

        /********************************************
         *  Intake Servo Mechanism Controls *
         *******************************************/

        if (gamepad2.dpad_up) {
            claw.setPosition(0);
        }
        if (gamepad2.dpad_down) {
            claw.setPosition(1);
        }
        if (gamepad2.dpad_left) {
            claw.setPosition((0.5));
        }

        /*
        //If freights in the box notify driver
        if(robot.FreightDetect.getDistance(DistanceUnit.CM) <= 5){
            telemetry.addData("freight", "in the box !!");
        }
        else
        {
            telemetry.addData("No","Freight");
        }*/

        /********************************************
         * Carousel Movement Controls *
         *******************************************/
        //gamepad1 buttons controles carousel motor
        if (gamepad1.x)
            robot.carouselMotor.setPower(0.25);
        if (gamepad1.b)
            robot.carouselMotor.setPower(-0.25);
        if (gamepad1.a)
            robot.carouselMotor.setPower(0);
    }
}
