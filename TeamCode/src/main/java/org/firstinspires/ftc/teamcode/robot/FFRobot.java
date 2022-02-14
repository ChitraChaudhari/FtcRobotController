package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;

public class FFRobot {
    public MecanumDrive drive = new MecanumDrive();
    public void init(HardwareMap hwmap)
    {
        drive.init(hwmap);
    }
}
