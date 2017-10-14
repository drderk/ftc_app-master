package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.PinkNavigate;

/**
 * Created by Hanna on 10/7/2017.
 */

@TeleOp(name="Gripper", group="Pushbot")
public class Gripper extends LinearOpMode {

    //declare variables
    Hardware         robot   = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    BNO055IMU imu;
    int currentAngle;
    int stage = 0;
    boolean auto = true;

    VuMarks camera = new VuMarks();
    RelicRecoveryVuMark picturePos;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Motion Start
        /*
        Starts at bottom and open
        Button (a) opens/closes
        Button (b) lowers to starting position
        Button (x) raises height (1 cube)
        Button (y) rotates 180 degrees
         */
        while (auto) {
            currentAngle = (int)GetHeading();
            switch (stage) {
                case 0 : //Rotate the arm to be perpendicular to the floor
                    robot.rotaterelic.setPosition(0.25);
                    stage = 1;
                    break;

                case 1: //lower claw
                    robot.liftRelic.setPower(1);
                    stage = 2;
                    break;

                case 2: //open claw
                    robot.grab.setPosition(1);
                    stage = 3;
                    break;

                case 3: //grip relic
                    robot.grab.setPosition(0);
                    stage = 4;
                    break;

                case 4: //lift claw
                    robot.liftRelic.setPower(-1);
                    stage = 5;
                    break;

                case 5: //extend arm
                    robot.extend.setPower(1);
                    stage = 6;
                    break;

                case 6: //lower
                    robot.liftRelic.setPower(1);
                    stage = 7;
                    break;

                case 7: //release
                    robot.grab.setPosition(1);
                    break;
            }
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
   */
    public double GetHeading(){
        Orientation angles;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

}
