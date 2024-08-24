package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.LQRController;


@TeleOp
@Config
public class LQRMecanumDriveOpMode extends OpMode {

    private DcMotorEx rearLeft, rearRight, frontLeft, frontRight;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private IMU imu;
    private LQRController lqrController;
    private boolean useLQR = false;
    private boolean previousBState = false;


    @Override
    public void init() {
        initializeHardware(hardwareMap);
        lqrController = new LQRController();
    }

    private void updateLQRController() {
        lqrController = new LQRController();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (System.currentTimeMillis() % 500 == 0) {
            updateLQRController();
        }
//        SimpleMatrix state = getCurrentState();
//        SimpleMatrix lqrInput = lqrController.calculateLQRInput(state);

        boolean currentBState = gamepad1.b;
        if (currentBState && !previousBState) {
            useLQR = !useLQR;
        }
        previousBState = currentBState;


        double[] powers;
        SimpleMatrix lqrInput = null;
        if (useLQR) {
            SimpleMatrix state = getCurrentState();
            lqrInput = lqrController.calculateLQRInput(state);
            powers = blendControlInputs(x, y, rx, lqrInput);
        } else {
            powers = calculateMecanumWheelPowers(x, y, rx);
        }

        setMotorPowers(powers);

        telemetry.addData("Drive Mode", useLQR ? "LQR" : "Normal");
        telemetry.addData("Motor Powers", java.util.Arrays.toString(powers));
        telemetry.addData("LQR Input", lqrInput.toString());
        telemetry.addData("Current State", previousBState);
        telemetry.update();
    }

    private void initializeHardware(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        rearLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        rearRight = hardwareMap.get(DcMotorEx.class, "rightRear");
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");

        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeft.setPower(0);
        rearRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        leftEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftEncoder")));
        rightEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightEncoder")));
        frontEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontEncoder")));
    }

    private SimpleMatrix getCurrentState() {
        double leftPosition = leftEncoder.getPositionAndVelocity().position;
        double rightPosition = rightEncoder.getPositionAndVelocity().position;
        double frontPosition = frontEncoder.getPositionAndVelocity().position;
        double leftVelocity = leftEncoder.getPositionAndVelocity().velocity;
        double rightVelocity = rightEncoder.getPositionAndVelocity().velocity;
        double frontVelocity = frontEncoder.getPositionAndVelocity().velocity;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        return new SimpleMatrix(new double[][]{
                {leftPosition},
                {rightPosition},
                {frontPosition},
                {heading},
                {leftVelocity},
                {rightVelocity}
        });
    }


    private double[] blendControlInputs(double x, double y, double rx, SimpleMatrix lqrInput) {
        double[] manualPowers = calculateMecanumWheelPowers(x, y, rx);
        double[] lqrPowers = {lqrInput.get(0), lqrInput.get(1), lqrInput.get(2), lqrInput.get(3)};
        double blendFactor = 0.7; // Adjust this to change the blend between manual and LQR control

        double[] blendedPowers = new double[4];
        for (int i = 0; i < 4; i++) {
            blendedPowers[i] = blendFactor * manualPowers[i] + (1 - blendFactor) * lqrPowers[i];
            blendedPowers[i] = Range.clip(blendedPowers[i], -1.0, 1.0);
        }
        return blendedPowers;
    }

    private double[] calculateMecanumWheelPowers(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        return new double[]{frontLeftPower, backLeftPower, frontRightPower, backRightPower};
    }

    private void setMotorPowers(double[] powers) {
        frontLeft.setPower(powers[0]);
        rearLeft.setPower(powers[1]);
        frontRight.setPower(powers[2]);
        rearRight.setPower(powers[3]);
    }
}
