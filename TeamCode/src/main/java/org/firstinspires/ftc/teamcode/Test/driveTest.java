package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.FFRobot;

@TeleOp(name = "TestDrive")
public class driveTest extends OpMode {
    FFRobot robot = new FFRobot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robotCentricDrive();
    }

    public void robotCentricDrive(){
        //controlling mecanum drive with gamepad1 left and right joystick
        double drive = gamepad1.left_stick_y;          //drive front or back
        double strafe = gamepad1.left_stick_x * 1.1;    //drive right or left
        //multiplying the left X value by 1.1 to
        // counteract imperfect strafing.
        double turn = gamepad1.right_stick_x;           //turn clockwise or anticlockwise

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        //set the motors power
        robot.frontLeftMotor.setPower((drive+strafe+turn)/denominator);
        robot.backLeftMotor.setPower((drive-strafe+turn)/denominator);
        robot.frontRightMotor.setPower((drive-strafe-turn)/denominator);
        robot.backRightMotor.setPower((drive+strafe-turn)/denominator);
    }
}
