package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp
public class FieldCentricDrive extends LinearOpMode {

    //creating and initializing four dc motor objects for all the drive motors on robot
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    //imu variable
    private BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {
        double driveTurn;

        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot;
        double gamepadDegree;
        double robotDegree;
        double movementDegree;
        double gamepadXControl;
        double gamepadYControl;


        //hardware mapping dc motors to configuration files
        frontLeftMotor = hardwareMap.dcMotor.get("FrontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("BackLeft");
        frontRightMotor = hardwareMap.dcMotor.get("FrontRight");
        backRightMotor = hardwareMap.dcMotor.get("BackRight");

        //setting both the right motors in reverse direction
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) {
            driveTurn = gamepad1.left_stick_x;
            gamepadXCoordinate = gamepad1.right_stick_x;
            gamepadYCoordinate = -gamepad1.right_stick_y;
            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate,gamepadYCoordinate),0,1);

            gamepadDegree = Math.atan2(gamepadXCoordinate,gamepadYCoordinate);

            robotDegree = getAngle();

            movementDegree = gamepadDegree - robotDegree;

            gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;

            gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;

            frontRightMotor.setPower(gamepadYControl*Math.abs(gamepadYControl) - gamepadXControl*Math.abs(gamepadXControl) + driveTurn);
            backRightMotor.setPower(gamepadYControl*Math.abs(gamepadYControl) + gamepadXControl*Math.abs(gamepadXControl) + driveTurn);
            frontLeftMotor.setPower(gamepadYControl*Math.abs(gamepadYControl) + gamepadXControl*Math.abs(gamepadXControl) - driveTurn);
            backLeftMotor.setPower(gamepadYControl*Math.abs(gamepadYControl) - gamepadXCoordinate*Math.abs(gamepadXControl)-driveTurn);
        }
        telemetry.update();
    }

    void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
    }

    //allows us to quickly get our z angle
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
