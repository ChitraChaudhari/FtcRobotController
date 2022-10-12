package org.firstinspires.ftc.teamcode.Mechanisams;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SpinWheel extends OpMode {
    private DcMotor intake_motor = null;

    @Override
    public void init() {
        intake_motor = hardwareMap.dcMotor.get("intake_motor");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up)
        {
            intake_motor.setPower(1);
        }else if (gamepad1.dpad_right)
            intake_motor.setPower(0);
        else if (gamepad1.dpad_down)
            intake_motor.setPower(-1);
    }
}
