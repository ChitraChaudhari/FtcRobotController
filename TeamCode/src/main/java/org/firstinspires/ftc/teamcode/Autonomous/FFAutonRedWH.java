package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Vision.Barcode.LEFT;
import static org.firstinspires.ftc.teamcode.Vision.Barcode.MIDDLE;
import static org.firstinspires.ftc.teamcode.Vision.Barcode.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.Barcode;
import org.firstinspires.ftc.teamcode.Vision.Scanner;
import org.firstinspires.ftc.teamcode.robot.FFRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class FFAutonRedWH extends LinearOpMode {
    FFRobot robot = new FFRobot();

    OpenCvWebcam webcam;

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
        robot.pointTurn(25, 0.3);

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
        robot.pointTurn(-80, 0.3);
        //Move to right so robot is in align with barrier
        //robot.MoveRobotToPositionStrafe(0.3, 8);
        //Move forward and park completely in warehouse
        robot.MoveRobotToPosition(0.3, 15);
        robot.pointTurn(-60,0.3);
        robot.MoveRobotToPosition(0.3, -2.5);
        robot.MoveRobotToPositionStrafe(0.3, 9);
    }
}
