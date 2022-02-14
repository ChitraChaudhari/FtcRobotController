package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;

public  class FFRobot {
    HardwareMap hwMap = null;

    //creating and initializing four dc motor objects for all the drive motors on robot
    public DcMotor frontLeftMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backRightMotor = null;

    public void init(HardwareMap ahwMap) {
        // save reference to hardware map
        hwMap = ahwMap;

        //hardware mapping dc motors to configuration files
        frontLeftMotor = hwMap.dcMotor.get("FrontLeft");
        backLeftMotor = hwMap.dcMotor.get("BackLeft");
        frontRightMotor = hwMap.dcMotor.get("FrontRight");
        backRightMotor = hwMap.dcMotor.get("BackRight");

        //setting both the right motors in reverse direction
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
    }
}
