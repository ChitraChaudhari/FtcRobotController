package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.FFRobot;

@Autonomous
public class IMUTutorial extends LinearOpMode {
    FFRobot robot = new FFRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

    }
}
