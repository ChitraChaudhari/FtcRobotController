package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.FFRobot;

@TeleOp(name = "FF Driver Control")

public class TeleopFF_READ_ME extends OpMode {

    FFRobot robot = new FFRobot();

    private double drive = 0.0;
    private double strafe = 0.0;
    private double turn = 0.0;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean freight = false;
    private boolean armLifted = false;
    private MecanumDrive m_drive;

    @Override
    public void init(){

        robot.init(hardwareMap);
        /*
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Arm Motor Encoders");
        telemetry.update();*/

        m_drive = new MecanumDrive();

        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.red.setMode(DigitalChannel.Mode.OUTPUT);
        robot.green.setMode(DigitalChannel.Mode.OUTPUT);
    }

    @Override
    public void loop() {

        /********************************
         * Mecanum Drive *
         *******************************/
        /*//controlling mecanum drive with gamepad1 left and right joystick
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

        robot.frontLeftMotor.setPower((drive+strafe+turn));
        robot.backLeftMotor.setPower((drive-strafe+turn));
        robot.frontRightMotor.setPower((drive-strafe-turn));
        robot.backRightMotor.setPower((drive+strafe-turn));

        robot.frontLeftMotor.setPower((drive + strafe + turn) / denominator);
        robot.backLeftMotor.setPower((drive - strafe + turn) / denominator);
        robot.frontRightMotor.setPower((drive - strafe - turn) / denominator);
        robot.backRightMotor.setPower((drive + strafe - turn) / denominator);*/

        m_drive.loop();

        /** FIX DRIVE TRAIN
         * Make sure the robot is moving according the the joystick
         * Fix the speed
         * Print the april tag value
         * Use the april tag value and create a auton man
         * Try to create an example auton path
         * */

//        /****************************************
//         *  Robot Arm Controls *
//         ***************************************/
//        //Gamepad2 left stick Y controls arm motor
//        //robot.ArmMotor.setPower(-gamepad2.left_stick_y);
//        //Gamepad2 buttons moves the robot arm to different positions
//        if (gamepad2.y) {
//            robot.driveArmToPosition(0.75, 1275);
//        }
//        if (gamepad2.a) {
//            robot.driveArmToPosition(0.75, 300);//
//        }
//        if (gamepad2.b) {
//            robot.driveArmToPosition(0.75, 750);
//        }
//        if (gamepad2.x) {
//            robot.driveArmToPosition(0.5, -300);
//            //robot.ArmMotor.setPower(0.0);
//        }
//        if (gamepad2.right_bumper) {
//            robot.ArmMotor.setPower(0.0);
//        }
//        //robot.ArmMotor.setPower(gamepad2.left_stick_y);
//
//        /********************************************
//         *  Turn table Mechanism Controls *
//         *******************************************/
//
//        robot.TurnTable.setPower(gamepad2.left_stick_x);
//
//        /********************************************
//         *  Intake Mechanism Controls *
//         *******************************************/
//
//        if (gamepad2.dpad_up) {
//            robot.IntakeMotor.setPower(0.8);
//        }
//        if (gamepad2.dpad_down) {
//            robot.IntakeMotor.setPower(-0.5);
//        }
//        if (gamepad2.dpad_left) {
//            robot.IntakeMotor.setPower(0);
//        }
//
//        /*
//        //If freights in the box notify driver
//        if(robot.FreightDetect.getDistance(DistanceUnit.CM) <= 5){
//            telemetry.addData("freight", "in the box !!");
//        }
//        else
//        {
//            telemetry.addData("No","Freight");
//        }*/
//
//        /********************************************
//         * Carousel Movement Controls *
//         *******************************************/
//        //gamepad1 buttons controles carousel motor
//        if (gamepad1.x == true)
//            robot.carouselMotor.setPower(0.25);
//        if (gamepad1.b == true)
//            robot.carouselMotor.setPower(-0.25);
//        if (gamepad1.a == true)
//            robot.carouselMotor.setPower(0);
//
//        if (robot.distance.getDistance(DistanceUnit.CM) < 10) {
//            robot.green.setState(true);
//            robot.red.setState(false);
//            freight = true;
//        } else {
//            robot.red.setState(true);
//            robot.green.setState(false);
//            freight = false;
//            armLifted = false;
//            telemetry.addData("armlifted:",armLifted);
//        }
//        if (freight == true && armLifted == false){
//            robot.driveArmToPosition(0.75, 500);
//            armLifted = true;
//            telemetry.addData("armlifted:", armLifted);
//        }
        telemetry.update();
    }
}

