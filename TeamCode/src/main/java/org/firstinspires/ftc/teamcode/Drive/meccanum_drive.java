// Mecanum drive using gamepad
// reference : https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum Drive custom chassis")
public class meccanum_drive extends OpMode {

    //creating and initializing four dc motor objects for all the drive motors on robot
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void init() {
        //hardware mapping dc motors to configuration files
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        //setting both the front right only motor in reverse direction
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        //controlling mecanum drive with gamepad1 left and right joystick
        double drive = gamepad1.left_stick_y;          //drive front or back
        double strafe = -(gamepad1.left_stick_x * 1.1);    //drive right or left
        //multiplying the left X value by 1.1 to
        // counteract imperfect strafing.
        double turn = gamepad1.right_stick_x;           //turn clockwise or anticlockwise

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(drive)+Math.abs(strafe)+Math.abs(turn),1);

        //set the motors power
        frontLeftMotor.setPower((drive+strafe+turn)/denominator);
        backLeftMotor.setPower((drive-strafe+turn)/denominator);
        frontRightMotor.setPower((drive-strafe-turn)/denominator);
        backRightMotor.setPower((drive+strafe-turn)/denominator);
    }
}

