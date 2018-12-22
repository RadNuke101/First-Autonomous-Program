package org.firstinspires.ftc.teamcode.ftclib.sample.opmode;

import android.os.PowerManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm;
import org.firstinspires.ftc.teamcode.ftclib.internal.controller.FinishableIntegratedController;
import org.firstinspires.ftc.teamcode.ftclib.internal.controller.PIDController;
import org.firstinspires.ftc.teamcode.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.ftclib.internal.sensor.IntegratingGyroscopeSensor;

/**
 * Created by Michaela on 1/3/2018.
 * Demonstrates the use of a HeadingableMecanumDrivetrain to autonomously rotate a robot to specific headings and hold those headings.
 * Tested and found fully functional by Gabriel on 2018-8-4.
 */

@Disabled
@Autonomous(name = "Headingable Mecanum Rotation Autonomous", group = "sample")

public class HeadingableMecanumRotationAutonomous extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor latch;

    final double COUNTS_PER_MOTOR_REV = 1440;
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double wheelDiam = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION)/(wheelDiam * 3.1415);

    public HeadingableMecanumDrivetrain drivetrain;

    public FinishableIntegratedController controller;

    public BNO055IMUImpl imu;

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {   //Notice that this is almost the exact same code as in HeadingableOmniwheelRotationAutonomous.
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
    frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        latch = hardwareMap.get(DcMotor.class, "Orbital20");

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //Add calibration file?
        parameters.loggingEnabled = true;   //For debugging
        parameters.loggingTag = "IMU";      //For debugging
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();  //Figure out why the naive one doesn't have a public constructor
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());


        PIDController pid = new PIDController(1.5, 0.05, 0);
        pid.setMaxErrorForIntegral(0.002);

        controller = new FinishableIntegratedController(new IntegratingGyroscopeSensor(imu), pid, new ErrorTimeThresholdFinishingAlgorithm(Math.PI/50, 1));
        drivetrain = new HeadingableMecanumDrivetrain(new DcMotor[]{frontLeft,frontRight, backLeft, backRight}, controller);
        for (DcMotor motor : drivetrain.motors) motor.setDirection(DcMotor.Direction.REVERSE);  //Depending on the design of the robot, you may need to comment this line out.
        for (DcMotor motor : drivetrain.motors) motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();

        drivetrain.setTargetHeading(Math.PI/2);
        while (drivetrain.isRotating()) {
            drivetrain.updateHeading();
            doTelemetry();
        }
        //drivetrain.rotate();
        sleep(1000);

        drivetrain.setTargetHeading(-Math.PI/2);
        while (drivetrain.isRotating()) {
            drivetrain.updateHeading();
            telemetry.addData("Heading", drivetrain.getCurrentHeading());
            telemetry.update();
        }
        sleep(1000);

        drivetrain.setTargetHeading(0);
        while (opModeIsActive()) drivetrain.updateHeading();

        inchDrive(1, 3, 3, 3, 3, 10.0);

        latch.setDirection(DcMotorSimple.Direction.REVERSE);
        latch.setPower(1);
        sleep(2000);

        //Strafs to the right
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        frontLeft.setPower(-0.5);
        backRight.setPower(-0.5);


        inchDrive(5,-5, -5, -5, -5, 5); //moves backwards
        inchDrive(2, -12, 12, 12, -12, 5); //turns backwards right
        inchDrive(5, -5,10, 10, -5, 5); //turns left






    }

    public void inchDrive(double speed, double leftFrunt, double rightFrunt, double rightBak, double leftBak,   double timeoutS){

        if(opModeIsActive()){
            int leftFruntTarg = frontLeft.getCurrentPosition() + (int)(leftFrunt * COUNTS_PER_INCH);
            int rightFruntTarg = frontRight.getCurrentPosition() + (int)(rightFrunt * COUNTS_PER_INCH);
            int leftBackTarg = backLeft.getCurrentPosition() + (int)(leftBak * COUNTS_PER_INCH);
            int rightBackTarg = backRight.getCurrentPosition() + (int)(rightBak * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(leftFruntTarg);
            backLeft.setTargetPosition(leftBackTarg);
            backRight.setTargetPosition(rightBackTarg);
            frontRight.setTargetPosition(rightFruntTarg);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));


            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);


            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    void doTelemetry() {
        PIDController pid = (PIDController) drivetrain.controller.algorithm;
        telemetry.addData("heading, target", drivetrain.controller.getSensorValue()+","+pid.getTarget());
        telemetry.addData("KP", pid.getKP());
        telemetry.addData("KI", pid.getKI());
        telemetry.addData("KD", pid.getKD());
        telemetry.addData("error", pid.getError());
        telemetry.addData("integral", pid.getIntegral());
        telemetry.addData("derivative", pid.getDerivative());
        telemetry.update();
    }
}
