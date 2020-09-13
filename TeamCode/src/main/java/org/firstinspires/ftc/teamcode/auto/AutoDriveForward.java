package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutoDriveForward extends LinearOpMode {
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction, power = 0.5;
    DcMotor leftFrontDrive, leftRearDrive,
            rightFrontDrive, rightRearDrive,
            intake, indexer, shooter, wobble;

    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        wobble = hardwareMap.get(DcMotor.class, "wobble");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure imu is calibrated
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();
        sleep(1000);

        while(opModeIsActive()) {
            correction = checkDirection();

            telemetry.addData("1: IMU heading", lastAngles.firstAngle);
            telemetry.addData("2: Global heading", globalAngle);
            telemetry.addData("3: Correction", correction);
            telemetry.update();
        }
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if(deltaAngle < -180) {
            deltaAngle += 360;
        }
        else if(deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    private double checkDirection() {
        double correction, angle, gain = 0.10;

        angle = getAngle();
        if(angle == 0) {
            correction = 0;
        }
        else {
            correction = -angle;
        }
        correction = correction * gain;
        return correction;
    }

    private void rotate(int degrees, double power) {
        double leftPower, rightPower;

        resetAngle();
        if(degrees < 0) {
            leftPower = power;
            rightPower = -power;
        }
        else if(degrees > 0) {
            leftPower = -power;
            rightPower = power;
        }
        else return;

        leftFrontDrive.setPower(leftPower);
        leftRearDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightRearDrive.setPower(rightPower);

        if(degrees < 0) {
            while(opModeIsActive() && getAngle() == 0) {}

            while(opModeIsActive() && getAngle() > degrees) {}
        }
        else {
            while(opModeIsActive() && getAngle() < degrees) {}
        }

        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);

        sleep(1000);
        resetAngle();
    }
}
