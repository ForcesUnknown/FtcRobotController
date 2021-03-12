package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;


@Autonomous(name="Autonomous", group="Autonomous")
public class AutonomousProgram extends RobotFunctions
{
    int captureCounter = 0;

    private ElapsedTime runtime = new ElapsedTime();

    private DriveBaseData driveBaseData = null;
    private DcMotorEx shooterMotor = null;
    private DcMotor wobbleMotor;

    private ServoData wobbleServo = null;
    private ServoData ringServoArm = null;

    private WebcamData webcamData = null;

    private IMUData imuData;
    private DigitalChannel touchSensor;

    private NormalizedColorSensor colourSensor;

    private int ringFlickTime = 1500;
    private int targetSquare = 0; // 0 -> A, 1 -> B, 2 -> C
    /*
        C
     B
        A
     */
    private final int hueLow = 5, hueHigh = 30, satLow = 128, satHigh = 255, valLow = 100, valHigh = 255; // colour ranges, 0 - 255
    private final int leftX = 700, topY = 600, rightX = 1000, bottomY = 750; //coords in picture to look at (doesnt need to process the edges. eg: dont check the walls)
    private final int lowThreshHold = 5000, highThreshHold = 30000; // 0 rings < low < 1 ring < high < 4 ring

    private BNO055IMU imu = null;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing, do not press play");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //region init

        driveBaseData = new DriveBaseData("LeftFront","RightFront","LeftBack", "RightBack", 75, 1440, hardwareMap);

        driveBaseData.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        colourSensor = hardwareMap.get(NormalizedColorSensor.class, "ColourSensor");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        wobbleMotor = hardwareMap.get(DcMotor.class, "WobbleMotor");
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ringServoArm = new ServoData("RingServoArm", 0.0, 0.2, hardwareMap, Servo.Direction.FORWARD);
        wobbleServo = new ServoData("WobbleServoArm", 0.5, 0.0, hardwareMap, Servo.Direction.FORWARD);

        imuData = new IMUData("imu", hardwareMap);

        //Webcam stuff
        webcamData = new WebcamData(hardwareMap, telemetry);


        sleep(3000);

        Bitmap picture = webcamData.GetImage();
        saveBitmap(picture);

        webcamData.CloseCamera();

        float color = Color.red(picture.getPixel(0, 0));

        int counter = 0;
        for (int x = leftX; x < rightX; x+=1) {
            for (int y = topY; y < bottomY; y+=1) {
                float[] HSV = new float[3];
                Color.colorToHSV(picture.getPixel(x, y), HSV);

                HSV[0] = (HSV[0] / 360) * 255;
                HSV[1] *= 255;
                HSV[2] *= 255;

                if(HSV[0] > hueLow && HSV[0] < hueHigh && HSV[1] > satLow && HSV[1] < satHigh && HSV[2] > valLow && HSV[2] < valHigh)
                {
                    counter++;
                }
            }
        }

        if(counter < lowThreshHold)
            targetSquare = 0;
        if(counter >= lowThreshHold && counter <= highThreshHold)
            targetSquare = 1;
        if(counter > highThreshHold)
            targetSquare = 2;

        if(picture == null)
            telemetry.addData("Status", "Failure: Webcam was not able to get an image, try reinitializing");
        else
            telemetry.addData("Status", "Initialized, you can now press play");

        telemetry.addLine("Orange Pixel Counter: " + counter);
        telemetry.addLine("Targeted Square: " + targetSquare + " (0 = A, 1 = B, 2 = C)");

        telemetry.addLine("Angle: " + imuData.HeadingAngle());

        telemetry.update();

        //endregion

        waitForStart();
        runtime.reset();


        DriveFrontBackDistance(driveBaseData, 1, 1371, 10);
        sleep(1000);
        DriveLeftRightDistance(driveBaseData, 1, 1000, 10);
        sleep(30000);


        switch(targetSquare)
        {
            case(0):
                Drop();
                DriveLeftRightDistance(driveBaseData, 1, -609, 10);
                TurnGyro(driveBaseData, 0.1, 0, imuData, 10);
                ShootRings();
                break;
            case(1):
                DriveLeftRightDistance(driveBaseData, 1, -609, 10);
                TurnGyro(driveBaseData, 0.1, 0, imuData, 10);
                ShootRings();
                DriveFrontBackDistance(driveBaseData, 1, 600, 10);
                Drop();
                break;
            case(2):
                DriveLeftRightDistance(driveBaseData, 1, -609, 10);
                TurnGyro(driveBaseData, 0.1, 0, imuData, 10);
                ShootRings();
                DriveFrontBackDistance(driveBaseData, 1, 1219, 10);
                DriveLeftRightDistance(driveBaseData, 1, 300, 10);
                Drop();
                break;

        }

        /** gyro vals
         * while(opModeIsActive())
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            Orientation a = imuData.Angles();

            telemetry.addData("Z", angles.firstAngle);
            telemetry.addData("Y", angles.secondAngle);
            telemetry.addData("X", angles.thirdAngle);

            telemetry.addData("_Z", a.firstAngle);
            telemetry.addData("_Y", a.secondAngle);
            telemetry.addData("_X", a.thirdAngle);

            telemetry.addData("_Z_", imuData.HeadingAngle());
            telemetry.update();
        }*/
        /**Encoder Test
        TurnMotorDistance(driveBaseData.rightBack, 1, 5, 30, 1440);
        TurnMotorDistance(driveBaseData.leftBack, 1, 5, 30, 1440);
        TurnMotorDistance(driveBaseData.rightFront, 1, 5, 30, 1440);
        TurnMotorDistance(driveBaseData.leftFront, 1, 5, 30, 1440);*/

        telemetry.addData("Motor", driveBaseData.leftBack.getCurrentPosition());

        telemetry.update();
    }

    private void Drop()
    {
        TurnMotorTime(wobbleMotor, 0.25, 100);
        SetServoPosition(wobbleServo.servo, wobbleServo.targetPosition);
        sleep(250);
        TurnMotorTime(wobbleMotor, -0.25, 100);
    }

    private void ShootRings()
    {
        shooterMotor.setPower(0.75);
        sleep(4000);

        for (int i = 0; i < 3; i++)
        {
            SetServoPosition(ringServoArm.servo, ringServoArm.targetPosition);
            sleep(ringFlickTime);
            SetServoPosition(ringServoArm.servo, ringServoArm.startPosition);
            sleep(ringFlickTime);
        }

        shooterMotor.setPower(0);
    }


    //region util
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    private void saveBitmap(Bitmap bitmap) {
        if(bitmap == null)
        {
            return;
        }

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);


        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
            telemetry.addLine("exception saving %s" + file.getName());
            telemetry.update();
        }
    }
    //endregion
}