package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Vision.Barcode.LEFT;
import static org.firstinspires.ftc.teamcode.Vision.Barcode.MIDDLE;
import static org.firstinspires.ftc.teamcode.Vision.Barcode.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Sensors.TurnPIDController;
import org.firstinspires.ftc.teamcode.Vision.Barcode;
import org.firstinspires.ftc.teamcode.Vision.Scanner;
import org.firstinspires.ftc.teamcode.robot.FFRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Autonomous
public class FFAutonRedWH_GYRO extends LinearOpMode {
    FFRobot robot = new FFRobot();

    private OpenCvWebcam webcam;

    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private Orientation lastAngle = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Scanner scanner = new Scanner(telemetry);
        webcam.setPipeline(scanner);

        // Webcam Streaming
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addData("robot", "press play to start");
        telemetry.update();

        waitForStart();

        Barcode result = scanner.getResult();

        webcam.stopStreaming();
        //Barcode result = MIDDLE;

        telemetry.addData("result",result);
        sleep(500);

        //Turn robot so the front of robot points to shipping hub
        telemetry.addData("Turning", "robot");
        telemetry.update();
        //robot.pointTurn(25, 0.3);
        turnPID(45);

        telemetry.addData("robot", "moving arm");
        telemetry.update();

        if(result == LEFT){
            robot.driveArmToPosition(0.75, 650);
            //Move Robot forward to shipping hub
            robot.MoveRobotToPosition(0.3, 8.25);
        }
        else if(result == MIDDLE) {
            robot.driveArmToPosition(0.75, 1100);
            //Move Robot forward to shipping hub
            robot.MoveRobotToPosition(0.3, 8.5);
        }
        else if(result == RIGHT){
            robot.driveArmToPosition(0.75, 1600);
            //Move Robot forward to shipping hub
            robot.MoveRobotToPosition(0.3, 9.75);
        }
        else{
            robot.driveArmToPosition(0.75, 1600);
            //Move Robot forward to shipping hub
            robot.MoveRobotToPosition(0.3, 9.75);
        }

        //Drop Freight
        robot.IntakeMotor.setPower(-0.5);
        sleep(500);
        robot.IntakeMotor.setPower(0.0);
        //Move robot little backwards
        robot.MoveRobotToPosition(0.3, -2);
        // Turn robot so it points straight towards ware house
        //robot.pointTurn(-80, 0.3);
        turnPID(-125);
        //Move to right so robot is in align with barrier
        //robot.MoveRobotToPositionStrafe(0.3, 8);
        //Move forward and park completely in warehouse
        robot.MoveRobotToPosition(0.3, 15);
        //robot.pointTurn(-60,0.3);
        turnPID(-90);
        robot.MoveRobotToPosition(0.3, -2.5);
        robot.MoveRobotToPositionStrafe(0.3, 9);
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
        runtime.reset();
        runtime.startTime();
        while (Math.abs(targetAngle-getAbsoluteAngle())> 3 || pid.getLastSlope() >0.75){
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            if(runtime.time(TimeUnit.MILLISECONDS) > 1000){
                break;
            }

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
