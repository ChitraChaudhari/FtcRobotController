package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.FFRobot;

public class driveTest extends OpMode {
    FFRobot robot = new FFRobot();
    HardwareMap hwMap = null;

    @Override
    public void init() {
        robot.init(hwMap);
    }

    @Override
    public void loop() {
        robot.drive.robotCentricDrive(gamepad1);
    }
}
