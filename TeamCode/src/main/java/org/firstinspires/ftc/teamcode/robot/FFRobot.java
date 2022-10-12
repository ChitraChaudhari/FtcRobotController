package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sensors.TurnPIDController;

import java.util.concurrent.TimeUnit;

public  class FFRobot {
    HardwareMap hwMap = null;

    //creating and initializing four dc motor objects for all the drive motors on robot
    public DcMotor frontLeftMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backRightMotor = null;

    public DcMotor ArmMotor = null;
    public DcMotor IntakeMotor = null;
    public DcMotor TurnTable = null;
    public DcMotor carouselMotor = null;

    public DigitalChannel red,green;
    public DistanceSensor distance;

    static final double TRACK_WIDTH = 11.0;             //distance between center of right and left wheel
    static final double COUNTS_PER_MOTOR_REV = 1680;    //Neverest 60 gear motor
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    private Orientation lastAngle = new Orientation();
    private double currAngle = 0.0;
    private ElapsedTime runtime = new ElapsedTime();


    // The IMU sensor object
    public BNO055IMU imu;

    public void init(HardwareMap ahwMap) {
        // save reference to hardware map
        hwMap = ahwMap;

        //hardware mapping dc motors to configuration files
        frontLeftMotor = hwMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hwMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hwMap.dcMotor.get("frontRightMotor");
        backRightMotor = hwMap.dcMotor.get("backRightMotor");

        ArmMotor = hwMap.get(DcMotor.class, "robot_arm_motor");
        IntakeMotor = hwMap.get(DcMotor.class, "intake_motor");
        carouselMotor = hwMap.get(DcMotor.class, "Carousel_DC");
        TurnTable = hwMap.get(DcMotor.class, "tt_Motor");

        red = hwMap.get(DigitalChannel.class, "red");
        green = hwMap.get(DigitalChannel.class, "green");
        distance = hwMap.get(DistanceSensor.class, "Distance sensor");


        //setting both the right motors in reverse direction
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set all motor powers to zer0
        setAllPower(0);
        //set drive motors to RUN_WITHOUT_ENCODER mode
        //set it to RUN_USING_ENCODERS if encoders needs to initialize
        setDriveMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public  void setAllPower(double p){setMotorPower(p,p,p,p);}

    public void setMotorPower(double lf,double rf,double lb, double rb)
    {
        frontLeftMotor.setPower(lf);
        backLeftMotor.setPower(lb);
        frontRightMotor.setPower(rf);
        backRightMotor.setPower(rb);
    }

    public void setDriveMotorMode(DcMotor.RunMode mode){
        frontLeftMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    //public void setAllPower(double p) {setMotorPower(p,p,p,p);}

    /*public void setMotorPower(double lF, double rF, double rB, double lB){
        frontLeftMotor.setPower(lF);
        frontRightMotor.setPower(rF);
        backRightMotor.setPower(rB);
        backLeftMotor.setPower(lB);
    }*/

    public void setMotorEncoderPosition(int flposition,int blfposition, int frposition,int brposition){
        frontLeftMotor.setTargetPosition(flposition);
        backLeftMotor.setTargetPosition(blfposition);
        backRightMotor.setTargetPosition(brposition);
        frontRightMotor.setTargetPosition(frposition);
    }

    public void setMotorModeRunToPosition()
    {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMotorModeStopAndResetEncoder()
    {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMotorModeRunUsingEncoder()
    {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MoveRobotToPositionStrafe(double power, double distanceInches){
        int newTargetPosition = (int)(distanceInches*COUNTS_PER_INCH) ;
        setMotorModeStopAndResetEncoder();
        setMotorEncoderPosition(-newTargetPosition, newTargetPosition, newTargetPosition, -newTargetPosition);
        setMotorModeRunToPosition();
        setMotorPower(power,-power,power,-power);
        while (backLeftMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy() ) {
        }
    }

    public void pointTurn(double angle, double power){
        // Determine new target position, and pass to motor controller
        double dist = (TRACK_WIDTH*Math.PI)*(angle/360);
        int ticks = (int) ((dist/(Math.PI*WHEEL_DIAMETER_INCHES))*COUNTS_PER_MOTOR_REV);
        setMotorModeStopAndResetEncoder();
        setMotorEncoderPosition(-ticks,-ticks,ticks,ticks);
        // Turn on RUN_TO_POSITION
        setMotorModeRunToPosition();
        //start the motion
        setMotorPower(power,power,power,power);
        // keep looping while we are still active, and there is time left, and all motors are running.
        while (backLeftMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy() ){
        }
        // Stop all motion;
        setAllPower(0.0);
        // Turn off RUN_TO_POSITION
        setMotorModeRunUsingEncoder();
    }

    public void driveArmToPosition(double power, int armPosition) {
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setTargetPosition(armPosition);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(power);
        /*
        while (ArmMotor.isBusy()) {
        }*/
    }

    public void MoveRobotToPosition(double power, double distanceInches) {
        int newTargetPosition;
        // Determine new target position, and pass to motor controller
        newTargetPosition = (int) (distanceInches * COUNTS_PER_INCH);
        setMotorModeStopAndResetEncoder();
        setMotorEncoderPosition(-newTargetPosition, -newTargetPosition, -newTargetPosition, -newTargetPosition);
        // Turn On RUN_TO_POSITION
        setMotorModeRunToPosition();
        //start motion
        setMotorPower(power, power, power, power);
        // keep looping while we are still active, and there is time left, and all motors are running.
        while (backLeftMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()) {
        }
        // Stop all motion;
        setAllPower(0.0);
        // Turn off RUN_TO_POSITION
        setMotorModeRunUsingEncoder();
    }

    public void MoveRobotToPositionStrafeRight(double power, double distanceInches){
        int newTargetPosition = (int)(distanceInches*COUNTS_PER_INCH) ;
        setMotorModeStopAndResetEncoder();
        setMotorEncoderPosition(-newTargetPosition, newTargetPosition, newTargetPosition, -newTargetPosition);
        setMotorModeRunToPosition();
        setMotorPower(power,power,power,power);
        while (backLeftMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy() ){
        }
    }

    public void MoveRobotToPositionStrafeLeft(double power, double distanceInches){
        int newTargetPosition = (int)(distanceInches*COUNTS_PER_INCH) ;
        setMotorModeStopAndResetEncoder();
        setMotorEncoderPosition(-newTargetPosition, newTargetPosition, newTargetPosition, -newTargetPosition);
        setMotorModeRunToPosition();
        setMotorPower(power,power,power,power);
        while (backLeftMotor.isBusy() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy() ){
        }
    }
    public void resetAngle(){
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0.0;
    }

    public double getAngle(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngle.firstAngle;

        if(deltaAngle > 180){
            deltaAngle -= 360;
        }else if(deltaAngle <= -360){
            deltaAngle += 360;
        }
        currAngle += deltaAngle;
        lastAngle = orientation;

        return currAngle;
    }

    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (Math.abs(error)>2){
            double motorPower = (error<0 ? -0.3 : 0.3);
            setMotorPower(-motorPower,motorPower,-motorPower,motorPower);
            error = degrees - getAngle();
        }

        setAllPower(0);
    }

    public void turnTo(double degrees){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
        return imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    public void turnToPID(double targetAngle){
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01,0,0.003);
        runtime.reset();
        runtime.startTime();
        while (Math.abs(targetAngle-getAbsoluteAngle())> 3 || pid.getLastSlope() >0.75){
            double motorPower = pid.update(getAbsoluteAngle());
            setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            if(runtime.time(TimeUnit.MILLISECONDS) > 1000){
                break;
            }

        }
        setAllPower(0);
    }
}




