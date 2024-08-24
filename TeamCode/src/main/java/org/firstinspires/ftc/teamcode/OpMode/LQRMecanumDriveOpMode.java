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

    private DcMotorEx backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor;
    private Encoder rightEncoder, frontEncoder;
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
        telemetry.addData("LQR Input", lqrInput != null ? lqrInput.toString() : "N/A");
        telemetry.addData("Current State", getCurrentState().toString());
        telemetry.update();
    }

    private void initializeHardware(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        rightEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightRear")));
        frontEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
    }

    private SimpleMatrix getCurrentState() {
        double rightPosition = rightEncoder.getPositionAndVelocity().position;
        double frontPosition = frontEncoder.getPositionAndVelocity().position;
        double rightVelocity = rightEncoder.getPositionAndVelocity().velocity;
        double frontVelocity = frontEncoder.getPositionAndVelocity().velocity;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        return new SimpleMatrix(new double[][]{
                {rightPosition},
                {frontPosition},
                {heading},
                {rightVelocity},
                {frontVelocity}
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
        frontLeftMotor.setPower(powers[0]);
        backLeftMotor.setPower(powers[1]);
        frontRightMotor.setPower(powers[2]);
        backRightMotor.setPower(powers[3]);
    }
}
