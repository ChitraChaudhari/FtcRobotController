package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Sensors.TurnPIDController;
import org.firstinspires.ftc.teamcode.robot.FFRobot;

@TeleOp
public class gyro extends LinearOpMode {
    FFRobot robot = new FFRobot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private Orientation lastAngle = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize the drive sysytem variables
        robot.init(hardwareMap);

        //send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        //wait for driver to press play
        waitForStart();
        /*
        turn(90);
        sleep(1000);
        turnTo(-90);
        */
        turnToPID(90);
        sleep(1000);
        turnToPID(-90);
    }

    public void resetAngle(){
        lastAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0.0;
    }

    public double getAngle(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngle.firstAngle;

        if(deltaAngle > 180){
            deltaAngle -= 360;
        }else if(deltaAngle <= -360){
            deltaAngle += 360;
        }
        currAngle += deltaAngle;
        lastAngle = orientation;

        telemetry.addData("gyro", orientation.firstAngle);

        return currAngle;
    }

    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error)>2){
            double motorPower = (error<0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower,motorPower,-motorPower,motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.setAllPower(0);
    }

    public void turnTo(double degrees){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //how far robot is turned to get to target position
        double error = degrees - orientation.firstAngle;

        if (error >360){
            error -= 360;
        }else if(error <-360){
            error += 360;
        }

        turn(error);
    }

    public double getAbsoluteAngle(){
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
    }

    void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    void turnToPID(double targetAngle){
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01,0,0.003);
        telemetry.setMsTransmissionInterval(50);
        while (opModeIsActive() && (Math.abs(targetAngle-getAbsoluteAngle())> 3 || pid.getLastSlope() >0.75)){
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllPower(0);
        telemetry.addData("turn", "complete");
        telemetry.update();
    }
}
