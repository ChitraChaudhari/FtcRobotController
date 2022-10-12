package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Vision.Barcode.LEFT;
import static org.firstinspires.ftc.teamcode.Vision.Barcode.MIDDLE;
import static org.firstinspires.ftc.teamcode.Vision.Barcode.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.Barcode;
import org.firstinspires.ftc.teamcode.Vision.Scanner;
import org.firstinspires.ftc.teamcode.robot.FFRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class FFAutonRedDC_GYRO extends LinearOpMode {
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

        telemetry.addData("result", result);
        sleep(500);

        if (result == LEFT) {
            robot.driveArmToPosition(0.75, 660);
        } else if (result == MIDDLE) {
            robot.driveArmToPosition(0.75, 1100);
        } else if (result == RIGHT) {
            robot.driveArmToPosition(0.75, 1570);
        } else {
            robot.driveArmToPosition(0.75, 1570);
        }
        //Move robot towards carousal
        robot.MoveRobotToPosition(0.5, 1.3);
        robot.turnPID(-90);
        robot.MoveRobotToPosition(0.3, -7.5);
        //deliver the duck
        robot.carouselMotor.setPower(-0.25);
        sleep(4000);
        robot.carouselMotor.setPower(0);

        //Turn robot so the front of robot points to shipping hub
        robot.turnPID(35);

        //Move robot to shipping hub
        if (result == LEFT) {
            robot.MoveRobotToPosition(0.3, 21.75);
        } else if (result == MIDDLE) {
            robot.MoveRobotToPosition(0.3, 13.75);
        } else if (result == RIGHT) {
            robot.MoveRobotToPosition(0.3, 12.75);
        } else {
            robot.MoveRobotToPosition(0.3, 14);
        }

        //Drop Freight
        robot.IntakeMotor.setPower(-0.5);
        sleep(700);
        robot.IntakeMotor.setPower(0.0);

        //Park in the storage unit
        robot.turnPID(-35);
        robot.MoveRobotToPosition(0.5, -12);
    }
}
