package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Disabled
@TeleOp(name="TeleOp Wonky", group="TeleOp")

public class TeleOpWonky extends OpMode {
    //Drivetrain Motors//
    private DcMotor backLeft;
    private DcMotor backRight;

    //Other Motors
    private DcMotor collector;
    private DcMotor evangelino;

    //Servos
    private Servo flickyWrist;
    private Servo liftrawrh;

    //Variables//
    private double leftStick, rightStick;
    private double leftPower, rightPower;
    private double gasPedal;
    private double coarseDiff, fineDiff, calibToggle;
    private int driveSpeed, driveMode;

    //Objects//
    public ElapsedTime runtime = new ElapsedTime();


    public void init() {
        //Motors//
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        evangelino = hardwareMap.dcMotor.get("lift");
        liftrawrh = hardwareMap.servo.get("liftrawrh");
        flickyWrist = hardwareMap.servo.get("flicky");
        collector = hardwareMap.dcMotor.get("collector");

        backRight.setDirection(DcMotor.Direction.REVERSE);

        //Variables//
        calibToggle = 0;
        driveSpeed = 0;
        driveMode = 0;

        //Speed Offsets
        coarseDiff = .6;
        fineDiff = .3;
    }


    public void loop() {
        // TOGGLE BUETONS //
        if (gamepad1.a && (runtime.seconds() > calibToggle)) {
            calibToggle = runtime.seconds() + 1;
            ++driveSpeed;
        }
        if (gamepad1.x && (runtime.seconds() > calibToggle)) {
            driveMode = 0;
        }
        if (gamepad1.y && (runtime.seconds() > calibToggle)) {
            driveMode = 1;
        }
        if (gamepad1.b && (runtime.seconds() > calibToggle)) {
            driveMode = 2;
        }

        if (driveMode == 0) {
            //////////////////////////////////// ARCADE DRIVE //////////////////////////////////////
            leftStick = gamepad1.left_stick_y;
            rightStick = -gamepad1.right_stick_x;

            //Left Side
            if (Math.abs(rightStick) > 0.5) {
                leftPower = leftStick / 2 + rightStick / 2;
            } else {
                leftPower = leftStick + rightStick / 2;
            }

            //Right Side
            if (Math.abs(rightStick) > 0.5) {
                rightPower = leftStick / 2 - rightStick / 2;
            } else {
                rightPower = leftStick - rightStick / 2;
            }

            if (driveSpeed % 2 == 0) {
                telemetry.addData("Mode: ", "ARCADE");
                telemetry.addData("Speed: ", "NORMAL");
                telemetry.addData("Stick: ", "X = " + round(rightStick, 3) + ", Y = " + round(leftStick, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * coarseDiff);
                backRight.setPower(rightPower * coarseDiff);
            } else {
                telemetry.addData("Mode: ", "ARCADE");
                telemetry.addData("Speed: ", "SLOW");
                telemetry.addData("Stick: ", "X = " + round(rightStick, 3) + ", Y = " + round(leftStick, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * fineDiff);
                backRight.setPower(rightPower * fineDiff);
            }
        } else if (driveMode == 1) {
            ///////////////////////////////////// TANK DRIVE ///////////////////////////////////////
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

            if (driveSpeed % 2 == 0) {
                telemetry.addData("Mode: ", "TANK");
                telemetry.addData("Speed: ", "NORMAL");
                telemetry.addData("Stick: ", "X = " + round(rightStick, 3) + ", Y = " + round(leftStick, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * coarseDiff);
                backRight.setPower(rightPower * coarseDiff);
            } else {
                telemetry.addData("Mode: ", "TANK");
                telemetry.addData("Speed: ", "SLOW");
                telemetry.addData("Stick: ", "X = " + round(rightStick, 3) + ", Y = " + round(leftStick, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * fineDiff);
                backRight.setPower(rightPower * fineDiff);
            }
        } else if (driveMode == 2) {
            ////////////////////////////////// ACKERMAN DRIVE //////////////////////////////////////
            leftStick = (-gamepad1.left_stick_x);

            gasPedal = (gamepad1.left_trigger - gamepad1.right_trigger);

            //Left Side
            if (Math.abs(rightStick) > 0.5) {
                leftPower = gasPedal / 2 + rightStick / 2;
            } else {
                leftPower = gasPedal + rightStick / 2;
            }

            //Right Side
            if (Math.abs(rightStick) > 0.5) {
                rightPower = gasPedal / 2 - rightStick / 2;
            } else {
                rightPower = gasPedal - rightStick / 2;
            }

            if (driveSpeed % 2 == 0) {
                telemetry.addData("Mode: ", "ACKERMAN");
                telemetry.addData("Speed: ", "NORMAL");
                telemetry.addData("Stick: ", "X = " + round(leftStick, 3) + ", G: " + round(gasPedal, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * coarseDiff);
                backRight.setPower(rightPower * coarseDiff);
            } else {
                telemetry.addData("Mode: ", "ACKERMAN");
                telemetry.addData("Speed: ", "SLOW");
                telemetry.addData("Stick: ", "L = " + round(leftStick, 3) + ", G: " + round(gasPedal, 3));
                telemetry.addData("Power: ", "L = " + round(leftPower, 3) + ", R = " + round(rightPower, 3));
                telemetry.update();

                backLeft.setPower(leftPower * fineDiff);
                backRight.setPower(rightPower * fineDiff);
            }
        }

        evangelino.setPower(-gamepad2.right_stick_y/2);

        if (gamepad2.dpad_up) {
            collector.setPower(0.60);
        } else if (gamepad2.dpad_down) {
            collector.setPower(-0.60);
        } else {
            collector.setPower(0.00);
        }

        if (gamepad2.right_bumper) {
            liftrawrh.setPosition(0.3);
        } else if (gamepad2.right_trigger > 0.7) {
            liftrawrh.setPosition(1.0);
        }

        if (gamepad2.y) {
            flickyWrist.setPosition(0.3);
        } else if (gamepad2.b) {
            flickyWrist.setPosition(0.62);
        }
        else if (gamepad2.a) {
            flickyWrist.setPosition(0.65);
        }
    }

    public static double round(double value, int places) { //Allows telemetry to display nicely
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
