/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous (name = "Auto Drive", group = "Pushbot")
@Disabled
public class AutoDrive extends LinearOpMode
{

    //declare variables
    public Hardware robot = new Hardware();   // Use a Pushbot's hardware
    public PinkNavigate PinkNavigate = new PinkNavigate();
    BNO055IMU imu;
    int currentAngle;
    int stage = 0;
    double leftMotorCmd, rightMotorCmd;
    //motor and servo setting values
    double collectPos = 0;
    double liftPos = 0;
    double jewelPos = 0;
    double grabPos = 0;
    double rotatePos = 0;
    double extendPos = 0;
    double targetPos = 0;
    double targetAngle = 0;
    RelicRecoveryVuMark image = null;
    //RelicRecoveryVuMark picturePos;
    private ElapsedTime runtime = new ElapsedTime();
    VuforiaTrackable relicTemplate;

    OpenGLMatrix lastLocation = null;
    RelicRecoveryVuMark vuMark = null;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode ()
    {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // See the calibration sample OpMode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        robot.init(hardwareMap);
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters cameraParameter = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        cameraParameter.vuforiaLicenseKey = "AaId2D7/////AAAAGeEOwvh4YkCKim1fP/VA8hpZk/olkYH12xynqz4wP+p4EjzCP4otFEoCxD0cztAeqHF3sVP3DJkAIOKqVX8UM6YbWpaaZPA3fK1YUfNg1Eh7A47eCRH0zO4hSJZ6fJEnw/NtT+dyv162iRX46R3xsyfB4CZdrHH2Yuxxoa9iWfaLfMdT7p7AWxUjHyujL28oC9xNcv2hJ0QDVbq3om6OzNEbAfkVbUf2q+z/VoWoH6036CL5fzB/ddo2E3Lgiv3PMoGtQyoWDtAuV6s53CAs/GuSGdv/WmltQtuxcu4w6QrdZIF2SCQ3idYKEPUuv16ranl1/Ayz5OgnYQf4HLRYLgnCRFKXEd7WZPVaLIwM9bJq"; //"ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";
        cameraParameter.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(cameraParameter);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY).
        telemetry.addData("heading", GetHeading());
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();
        relicTrackables.activate();
        //Motion Start
        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                telemetry.addData("Opmode:", "active");
                //telemetry.update();
                currentAngle = (int) GetHeading();
                switch (stage)
                {
                    case 0: //initialize
                        collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos = 0;
                        targetPos = 0;
                        targetAngle = 0;

                        stage = 10;
                        break;

                    case 10: //scan surroundings
                        collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos = 0;
                        targetPos = 0;
                        targetAngle = 0;
                        image = getImage();
                        // If the camera is seeing a known image
                        if (image != RelicRecoveryVuMark.UNKNOWN)
                        {
                            // change behavior based on pose
                            telemetry.addData("Vumarks", image);
                            stage = 30;
                        }
                        else
                        {
                            telemetry.addData("Vumarks", "No picture!");
                            stage = 10;
                        }
                        sleep(1000);
                        //telemetry.addData("Jewel Color", jewelColor());
                        break;

                    case 20: //lower the jewel arm
                        collectPos = 0;
                        liftPos = 0;
                        jewelPos = 1;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos = 0;
                        targetPos = 0;
                        targetAngle = 0;

                        sleep(1000);
                        telemetry.addData("Stage", stage);
                        stage = 30;
                        break;

/*                    case 30: //lift up jewel mechanism and drive forward
                        collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos = 0;
                        targetAngle = 0;

                        if(image == RelicRecoveryVuMark.CENTER){
                            targetPos = 25;
                        } else if (image == RelicRecoveryVuMark.LEFT){
                            targetPos = 20;
                        } else if (image == RelicRecoveryVuMark.RIGHT){
                            targetPos = 30;
                        } else {
                            targetPos = 0;
                        }

                        telemetry.addData("Stage", stage);
                        //drive completely off ramp
                        if (PinkNavigate.driveToPos(targetPos, -targetAngle, currentAngle, robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), 0, 0, 1))
                        {
                            stage = 40;
                        }
                        else
                        {
                            stage = 30;
                        }
                        break;

                    case 40: //turn
                        collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos = 0;
                        targetPos = 45;
                        targetAngle = 90;

                        telemetry.addData("Stage", stage);

                        if (PinkNavigate.driveToPos(targetPos, -targetAngle, currentAngle, robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), 0, 0, 1))
                        {
                            stage = 50;
                        }
                        else
                        {
                            stage = 40;
                        }
                        break;

                    case 50: //drive to center
                        collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos = 0;
                        targetPos = 90;
                        targetAngle = 90;

                        telemetry.addData("Stage", stage);
                        if (PinkNavigate.driveToPos(targetPos, -targetAngle, currentAngle, robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), 0, 0, 1))
                        {
                            stage = 100;
                        }
                        else
                        {
                            stage = 50;
                        }
                        break;
                    case 60: //drive to center
                        collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos = 0;
                        targetPos = 90;
                        targetAngle = 90;
                        telemetry.addData("Stage", stage);

                        if (PinkNavigate.driveToPos(targetPos, -targetAngle, currentAngle, robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), 0, 0, 1))
                        {
                            stage = 70;
                        }
                        else
                        {
                            stage = 60;
                        }
                        break;
                    case 70: //drive to center
                        collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos = 0;
                        targetPos = 90;
                        targetAngle = 90;
                        telemetry.addData("Stage", stage);

                        if (PinkNavigate.driveToPos(targetPos, -targetAngle, currentAngle, robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), 0, 0, 1))
                        {
                            stage = 100;
                        }
                        else
                        {
                            stage = 70;
                        }
                        break;

                    case 100: //end
                        collectPos = 0;
                        liftPos = 0;
                        jewelPos = 0;
                        grabPos = 0;
                        rotatePos = 0;
                        extendPos = 0;
                        PinkNavigate.stopBase();
                        telemetry.addData("Stage", "Complete");
                        break;
*/
                }

                //set all values
                robot.leftDrive.setPower(org.firstinspires.ftc.teamcode.PinkNavigate.getLeftMotorCmd());
                robot.rightDrive.setPower(org.firstinspires.ftc.teamcode.PinkNavigate.getRightMotorCmd());
               /* robot.collect.setPosition(collectPos);
                robot.lift.setPower(liftPos);
                robot.jewel.setPosition(jewelPos);
                robot.grab.setPosition(grabPos);
                robot.rotate.setPosition(rotatePos);
                robot.extend.setPower(extendPos);*/

                // readouts
                telemetry.addData("LinearError", targetPos - (robot.rightDrive.getCurrentPosition() + robot.leftDrive.getCurrentPosition()) / 2);
                telemetry.addData("AngularError", targetAngle - GetHeading());
                telemetry.addData("Angle", GetHeading());
                telemetry.update();
            }
            telemetry.update();

        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
   */

    public double GetHeading ()
    {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    public RelicRecoveryVuMark getImage ()
    {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN)
        {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null)
            {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
            return vuMark;
        }
        return vuMark;
    }

    private String format (OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
    /*public String jewelColor() {
        if (robot.colorSensor.red() > 3) {
            return "Red";
        }
        else{
            return "Blue";
        }*/
