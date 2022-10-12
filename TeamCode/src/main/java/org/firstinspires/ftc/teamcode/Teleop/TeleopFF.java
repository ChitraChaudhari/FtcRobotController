package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.FFRobot;
@Disabled
@TeleOp

public class TeleopFF extends OpMode {

    FFRobot robot = new FFRobot();

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles.
    //static final boolean FIELD_CENTRIC = false;
    private boolean FIELD_CENTRIC = false;
    private Motor fl,fr, bl, br;
    private MecanumDrive drive;
    private GamepadEx driveOp;

    RevIMU imu;

/*
    private double drive = 0.0;
    private double strafe = 0.0;
    private double turn = 0.0;*/
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(){

        robot.init(hardwareMap);
        /*
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Arm Motor Encoders");
        telemetry.update();*/

        fl =  new Motor(hardwareMap, "frontLeftMotor", 1680,105);
        fr =  new Motor(hardwareMap, "frontRightMotor", 1680, 105);
        bl =  new Motor(hardwareMap, "backLeftMotor", 1680, 105);
        br =  new Motor(hardwareMap, "backRightMotor", 1680, 105);

        br.setInverted(true);

        imu = new RevIMU(hardwareMap);
        imu.init();

        //the extended gamepad object
        driveOp = new GamepadEx(gamepad1);

        drive = new MecanumDrive(fl,fr,bl,br);

        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.red.setMode(DigitalChannel.Mode.OUTPUT);
        robot.green.setMode(DigitalChannel.Mode.OUTPUT);
    }

    @Override
    public void loop() {

        /********************************
         * Mecanum Drive *
         *******************************//*
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
        */

        if(gamepad1.back)
        {
            if(FIELD_CENTRIC == false){
                FIELD_CENTRIC = true;
            }else if(FIELD_CENTRIC == true){
                FIELD_CENTRIC = false;
            }
        }

        // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
        // These are related to the left stick x value, left stick y value, and
        // right stick x value respectively. These values are passed in to represent the
        // strafing speed, the forward speed, and the turning speed of the robot frame
        // respectively from [-1, 1].

        if(!FIELD_CENTRIC){
            // For a robot centric model, the input of (0,1,0) for (leftX, leftY, rightX)
            // will move the robot in the direction of its current heading. Every movement
            // is relative to the frame of the robot itself.
            //
            //                 (0,1,0)
            //                   /
            //                  /
            //           ______/_____
            //          /           /
            //         /           /
            //        /___________/
            //           ____________
            //          /  (0,0,1)  /
            //         /     ↻     /
            //        /___________/

            drive.driveRobotCentric(
                    driveOp.getLeftX(),
                    driveOp.getLeftY(),
                    driveOp.getRightX(),
                    false
            );
        }else {
            // Below is a model for how field centric will drive when given the inputs
            // for (leftX, leftY, rightX). As you can see, for (0,1,0), it will travel forward
            // regardless of the heading. For (1,0,0) it will strafe right (ref to the 0 heading)
            // regardless of the heading.
            //
            //                   heading
            //                     /
            //            (0,1,0) /
            //               |   /
            //               |  /
            //            ___|_/_____
            //          /           /
            //         /           / ---------- (1,0,0)
            //        /__________ /
            drive.driveFieldCentric(
                    driveOp.getLeftX(),
                    driveOp.getLeftY(),
                    driveOp.getRightX(),
                    imu.getRotation2d().getDegrees(),
                    false
            );
        }
        /****************************************
         *  Robot Arm Controls *
         ***************************************/
        //Gamepad2 left stick Y controls arm motor
        //robot.ArmMotor.setPower(-gamepad2.left_stick_y);
        //Gamepad2 buttons moves the robot arm to different positions
        /*if (gamepad2.y) {
            robot.driveArmToPosition(0.75, 1275);
        }*/
        if (gamepad2.a) {
            robot.driveArmToPosition(0.75, 300);//
        }
        if (gamepad2.b) {
            robot.driveArmToPosition(0.75, 800);
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
         *  Intake Mechanism Controls *
         *******************************************/

        if (gamepad2.dpad_up) {
            robot.IntakeMotor.setPower(0.8);
        }
        if (gamepad2.dpad_down) {
            robot.IntakeMotor.setPower(-0.5);
        }
        if (gamepad2.dpad_left) {
            robot.IntakeMotor.setPower(0);
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
        if (gamepad1.x == true)
            robot.carouselMotor.setPower(0.25);
        if (gamepad1.b == true)
            robot.carouselMotor.setPower(-0.25);
        if (gamepad1.a == true)
            robot.carouselMotor.setPower(0);

        if (robot.distance.getDistance(DistanceUnit.CM) < 10) {
            robot.green.setState(true);
            robot.red.setState(false);
        } else {
            robot.red.setState(true);
            robot.green.setState(false);
        }
    }
}
