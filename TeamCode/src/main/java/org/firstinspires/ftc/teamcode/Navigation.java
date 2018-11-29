package org.firstinspires.ftc.teamcode;
//EXIST
import android.content.Context;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.io.File;

/**
 * A class for all movement methods for Rover Ruckus.
 */
public class Navigation{

    //-----tweak values-----//
    private float maximumMotorPower = 0.5f;             //when executing a goToLocation function, robot will never travel faster than this value (percentage 0=0%, 1=100%)
    private float encoderCountsPerRev = 537.6f;         //encoder ticks per one revolution
    private boolean useTelemetry;                       //whether to execute the telemetry method while holding
    private float minVelocityCutoff = 0.05f;            //velocity with which to continue program execution during a hold (encoder ticks per millisecond)

    //-----enums-----//
    public enum CubePosition {UNKNOWN, LEFT, MIDDLE, RIGHT}
    private CubePosition cubePos = CubePosition.UNKNOWN;
    public enum CollectorHeight {COLLECT, HOLD, DUMP}
    public enum LiftHeight {LOWER, SCORE}
    public enum CollectorExtension {PARK, DUMP, OUT}
    public enum LiftLock {LOCK,UNLOCK}
    public enum CollectorSweeper {INTAKE,OUTTAKE, OFF}

    //-----robot hardware, position, and dimensions-----//
    private com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter;
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    private float wheelDistance = 6.66f;                //distance from center of robot to center of wheel (inches)
    private float wheelDiameter = 4;                //diameter of wheel (inches)
    private Location pos = new Location();           //location of robot as [x,y,z,rot] (inches / degrees)
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    //-----internal values-----//
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private Dogeforia vuforia;
    private GoldAlignDetector detector;
    private WebcamName webcamName;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private DcMotor velocityMotor;
    private long prevTime;
    private int prevEncoder;
    private float velocity = 0f;

    //-----motors-----//
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor extendy; //collector extension
    private DcMotor lifty;  //lift motor a
    private DcMotor liftyJr; //lift motor b

    //-----servos-----//
    private Servo droppy;  //collection lift a
    private Servo droppyJr; //collection lift b
    private CRServo collecty;  //collection sweeper
    private Servo liftyLock; //lift lock
    private Servo teamMarker;

    /**
     * The constructor class for Navigation
     * @param hardwareGetter - The OpMode required to access motors. Often, 'this' will suffice.
     * @param telemetry - Telemetry of the current OpMode, used to output data to the screen.
     * @param useTelemetry - Whether or not to output information about stored variables and motors during hold periods.
     */
    public Navigation(com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry telemetry, boolean useTelemetry) {
        this.hardwareGetter = hardwareGetter;
        this.telemetry = telemetry;
        this.useTelemetry = useTelemetry;

        //-----motors-----//
        frontLeft = hardwareGetter.hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareGetter.hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareGetter.hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareGetter.hardwareMap.dcMotor.get("backRight");
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendy = hardwareGetter.hardwareMap.dcMotor.get("extendy");
        extendy.setDirection(DcMotorSimple.Direction.REVERSE);
        extendy.setPower(1f);
        extendy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendy.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setCollectorExtension(0);

        lifty = hardwareGetter.hardwareMap.dcMotor.get("lifty");
        lifty.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifty.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifty.setPower(1);

        liftyJr = hardwareGetter.hardwareMap.dcMotor.get("liftyJr");
        liftyJr.setDirection(DcMotor.Direction.REVERSE);
        liftyJr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftyJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftyJr.setPower(1);
        setLiftHeight(0);

        //-----servos-----//
        teamMarker = hardwareGetter.hardwareMap.servo.get("teamMarker");
        liftyLock = hardwareGetter.hardwareMap.servo.get("liftyLock");
        collecty = hardwareGetter.hardwareMap.crservo.get("collecty");
        droppy = hardwareGetter.hardwareMap.servo.get("droppy");
        droppyJr = hardwareGetter.hardwareMap.servo.get("droppyJr");
        droppyJr.setDirection(Servo.Direction.REVERSE);

        //----Vuforia Params---///
        webcamName = hardwareGetter.hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia licence key
        parameters.vuforiaLicenseKey = " AYSaZfX/////AAABGZyGj0QLiEYhuyrGuO59xV2Jyg9I+WGlfjyEbBxExILR4A183M1WUKucNHp5CnSpDGX5nQ9OD3w5WCfsJuudFyJIJSKZghM+dOlhTWWcEEGk/YB0aOLEJXKK712HpyZqrvwpXOyKDUwIZc1mjWyLT3ZfCmNHQ+ouLKNzOp2U4hRqjbdWf1ZkSlTieiR76IbF6x7MX5ZtRjkWeLR5hWocakIaH/ZPDnqo2A2mIzAzCUa8GCjr80FJzgS9dD77lyoHkJZ/5rNe0k/3HfUZXA+BFSthRrtai1W2/3oRCFmTJekrueYBjM4wuuB5CRqCs4MG/64AzyKOdqmI05YhC1tVa2Vd6Bye1PaMBHmWNfD+5Leq ";
        parameters.fillCameraMonitorViewParent = true;

        // Set camera name for Vuforia config
        parameters.cameraName = webcamName;

        // Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();


        //Setup trackables
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        detector = new GoldAlignDetector();
        detector.init(hardwareGetter.hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

        //-----velocity control-----//
        velocityMotor = frontLeft;
        prevTime = System.currentTimeMillis();
        prevEncoder = velocityMotor.getCurrentPosition();
    }

    /**
     * Updates the cube location enumerator using OpenCV. Access using [nav].cubePos.
     * @return boolean, true if updated, false if not updated.
     */
    public boolean updateCubePos() {

        double cubeX = detector.getXPosition();
        //unknown
        if(cubeX == 0.0) {
            return false;
        }
        //left
        else if(cubeX < 170) {
            cubePos = CubePosition.LEFT;
        }
        //middle
        else if(cubeX < 400) {
            cubePos = CubePosition.MIDDLE;
        }
        //right
        else {
            cubePos = CubePosition.RIGHT;
        }


        return true;
    }

    /**
     * Returns the current stored cube position.
     * @return CubePosition enumerator. Locations self-explanatory.
     */
    public CubePosition getCubePos() {return cubePos;}


    /**
     * Sets drive motor powers.
     * @param left power of left two motors as percentage (0-1).
     * @param right power of right two motors as percentage (0-1).
     */
    public void drivePower(float left, float right) {
        frontLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
        backLeft.setPower(left);
    }

    /**
     * Sets drive motor target encoder to given values.
     * @param left encoder set for left motors.
     * @param right encoder set for right motors.
     */
    public void drivePosition(int left, int right) {
        frontLeft.setTargetPosition(left);
        frontRight.setTargetPosition(right);
        backRight.setTargetPosition(right);
        backLeft.setTargetPosition(left);
    }

    /**
     * Sets all drive motor run modes to given mode.
     * @param r DcMotor mode to given value.
     */
    public void driveMode(DcMotor.RunMode r) {
        frontLeft.setMode(r);
        frontRight.setMode(r);
        backRight.setMode(r);
        backLeft.setMode(r);
    }

    /**
     * Stops all drive motors and resets encoders.
     */
    public void stopAllMotors() {
        drivePower(0f,0f);
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Pseudo PID to drive the given distance.
     * @param distance Distance to drive forward in inches.
     */
    public void goDistance(float distance) {
        //driveMethodComplex(-distance, slowdown, 0f, frontLeft, 1f, 1f, false, minimumMotorPower, maximumMotorPower);
        driveMethodSimple(-distance, distance, maximumMotorPower, maximumMotorPower);
        pos.translateLocal(distance);
    }

    /**
     * Executes a point turn to face the given world rotation.
     * @param rot Target azimuth in degrees
     */
    public void pointTurn(float rot) {
        float rota = (rot - pos.getLocation(3)) % 360f;
        float rotb = -(360f - rota);
        float optimalRotation = (Math.abs(rota) < Math.abs(rotb) ? rota : rotb); //selects shorter rotation
        float distance = (float)(Math.toRadians(optimalRotation) * wheelDistance); //arc length of turn (radians * radius)

        //driveMethodComplex(distance, slowdown, precision, frontLeft, 1f, -1f, true, 0.05f, 0.25f);
        driveMethodSimple(distance, distance, 0.3f, 0.3f);


        pos.setRotation(rot);
    }

    /**
     * Executes a point turn to face the given Location.
     * @param loc Target Location object
     */
    public void pointTurn(Location loc) {
        pointTurn((float) Math.toDegrees(Math.atan2(loc.getLocation(2) - pos.getLocation(2), loc.getLocation(0) - pos.getLocation(0))));
    }

    /**
     * Executes a point turn relative to the current location. Positive is counterclockwise.
     * @param rot the amount to rotate the robot in degrees. Positive is counterclockwise.
     */
    public void pointTurnRelative(float rot) {
        pointTurn(pos.getLocation(3)+rot);
    }

    /**
     * Sets lift motor to given encoder position
     * @param position Encoder ticks for lift motor. ~0(bottom) to ~8200(top)
     */
    public void setLiftHeight(int position) {
        lifty.setTargetPosition(position);
        liftyJr.setTargetPosition(position);
    }

    /**
     * Sets the lift height to a pre-programmed position.
     * @param position LiftHeight enumerator. Options are LOWER, HOOK, or SCORE.
     */
    public void setLiftHeight(LiftHeight position) {
        switch(position) {
            case LOWER:
                setLiftHeight(0);
                break;
            case SCORE:
                setLiftHeight(8200);
                break;
        }
    }

    /**
     * Sets the collection sweeper to a given power value.
     * @param power float. Percentage power at which to run collector. 1.0f (intake) - -1.0f(outtake) inclusive.
     */
    public void setCollectionSweeper(float power) {
        collecty.setPower(power);
    }

    /**
     * Sets the collection sweeper power using pre-programmed values.
     * @param power CollectorSweeper emumerator. Options are INTAKE, OUTTAKE, or OFF.
     */
    public void setCollectionSweeper(CollectorSweeper power) {
        switch(power) {
            case INTAKE:
                setCollectionSweeper(0.5f);
                break;
            case OUTTAKE:
                setCollectionSweeper(-0.5f);
                break;
            case OFF:
                setCollectionSweeper(0f);
                break;
        }
    }

    /**
     * Set the height of the collector arm.
     * @param position float. ~0.8f (bottom) to ~0.18f (top).
     */
    public void setCollectorHeight(float position) {
        droppy.setPosition(position);
        droppyJr.setPosition(position);
    }

    /**
     * Sets the height of the collector arm.
     * @param position CollectorHeight enumerator. Options are COLLECT, HOLD, or DUMP.
     */
    public void setCollectorHeight(CollectorHeight position) {
        switch(position) {
            case COLLECT:
                setCollectorHeight(0.715f);
                break;
            case HOLD:
                setCollectorHeight(0.5f);
                break;
            case DUMP:
                setCollectorHeight(0.18f);
                break;
        }
    }

    /**
     * Sets the extension of the collector arm.
     * @param position int. 0(in) to 1600(out).
     */
    public void setCollectorExtension(int position) {
        extendy.setTargetPosition(position);
    }

    /**
     * Sets the extension of the collectior arm.
     * @param position CollectorExtension enumerator. Options are PARK, DUMP, or OUT.
     */
    public void setCollectorExtension(CollectorExtension position) {
        switch (position){
            case PARK:
                setCollectorExtension(0);
                break;
            case DUMP:
                setCollectorExtension(500);
                break;
            case OUT:
                setCollectorExtension(1600);
                break;
        }
    }

    /**
     * Sets the position of the teamMarker servo.
     * @param position float. 0.0f(locked) to 0.8f(dropping).
     */
    public void setTeamMarker(float position) {
        teamMarker.setPosition(position);
    }

    /**
     * Drive method that independantly controls the position and power of the left and right drive motors.
     * @param distanceL float. Distance in inches for left motors to traverse.
     * @param distanceR float. Distance in inches for right motors to traverse.
     * @param LPower float. Power percentage for left motors (0.0-1.0).
     * @param RPower float. Power percentage for right motors (0.0-1.0).
     */
    private void driveMethodSimple(float distanceL, float distanceR, float LPower, float RPower) {
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int l = (int)(distanceL / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        int r = (int)(distanceR / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        drivePosition(-l,-r);
        drivePower(LPower,RPower);
        driveMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Holds program execution until drive motor velocities are below the minimum cutoff.
     * Will output telemetry if class initialized with useTelemetry true.
     */
    public void holdForDrive() {
        hold(0.2f);
        while(updateVelocity() > minVelocityCutoff) {
            if(useTelemetry) telemetryMethod();
        }
    }

    /**
     * Hold program for given number of seconds.
     * @param seconds float. Number of seconds to wait.
     */
    public void hold(float seconds) {
        long stopTime = System.currentTimeMillis() + (long) (seconds * 1000);
        while (System.currentTimeMillis() < stopTime) {
            if (useTelemetry) telemetryMethod();
        }
    }

    /**
     * Updates the stored velocity of the robot to reflect reality.
     * @return float. New velocity in encoder ticks per millisecond.
     */
    private float updateVelocity(){
        velocity = Math.abs((float)(velocityMotor.getCurrentPosition() - prevEncoder) / (System.currentTimeMillis() - prevTime));
        prevEncoder = velocityMotor.getCurrentPosition();
        prevTime = System.currentTimeMillis();
        return velocity;
    }

    /**
     * A simple method to output the status of all motors and other variables to telemetry.
     */
    public void telemetryMethod() {
        updateVelocity();
        String motorString = "FL-" + frontLeft.getCurrentPosition() + " BL-" + backLeft.getCurrentPosition() + " FR-" + frontRight.getCurrentPosition() + " BR-" + backRight.getCurrentPosition();
        telemetry.addData("Drive", motorString);
        telemetry.addData("Lift",lifty.getCurrentPosition()+" " +liftyJr.getCurrentPosition());
        telemetry.addData("Collector L/E/C",lifty.getCurrentPosition()+" "+extendy.getCurrentPosition()+" "+collecty.getPower());
        telemetry.addData("Pos",pos);
        telemetry.addData("CubePos",cubePos);
        telemetry.addData("Velocity",velocity);
        telemetry.addData("CubeXPosition",detector.getXPosition());
        telemetry.update();
    }
}