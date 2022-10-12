package org.firstinspires.ftc.teamcode.Mechanisams;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawTest extends OpMode {
    Servo claw;
    DcMotor intakeArm;

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "ClawServo");
        intakeArm = hardwareMap.get(DcMotor.class, "intake_motor");
    }

    @Override
    public void loop() {
        intakeArm.setPower(gamepad1.right_stick_y);

        if(gamepad1.a){
            claw.setPosition(0);
        }
        if(gamepad1.b){
            claw.setPosition(1);
        }
        if(gamepad1.x){
            claw.setPosition((0.5));
        }
    }
}
