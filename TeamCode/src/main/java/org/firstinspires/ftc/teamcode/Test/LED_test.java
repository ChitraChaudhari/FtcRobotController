package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class LED_test extends LinearOpMode {
    DigitalChannel red,green;
    DistanceSensor distance;
    @Override
    public void runOpMode() throws InterruptedException {
        red = hardwareMap.get(DigitalChannel.class, "red");
        green = hardwareMap.get(DigitalChannel.class, "green");
        distance = hardwareMap.get(DistanceSensor.class, "Distance sensor");

        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);

        waitForStart();

        while (opModeIsActive()){
            if (distance.getDistance(DistanceUnit.CM) < 10){
                green.setState(true);
                red.setState(false);
            }
            else{
                red.setState(true);
                green.setState(false);
            }
        }
    }
}
