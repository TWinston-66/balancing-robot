package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class Hardware {
    // Declare Drive Motors
    public DcMotor LeftDrive = null; // Main Drive Motors
    public DcMotor RightDrive = null;

    // Declare Sensors
    BNO055IMU imu; // Embedded Control Hub imu
    //DigitalChannel duckTouchRed;

    // Declare led lights
    public RevBlinkinLedDriver lights;

    // Hardware Map
    HardwareMap hardwareMap = null;

    // Constructor Method
    public Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        initializeRobot();
    }

    // Private Methods
    private void initializeRobot() {
        initDriveMotors();;
        initGyro();
        initLights();
    }

    public void initGyro() {
        // IMU Parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private void initLights() {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led_lights");
    }

    private void initDriveMotors() {
        // Initialize Motors
        RightDrive = hardwareMap.get(DcMotor.class, "right");
        LeftDrive = hardwareMap.get(DcMotor.class, "left");


        // Main Drive Motor Direction
        RightDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        // Main Drive Motor Zero Power Behavior
        LeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
