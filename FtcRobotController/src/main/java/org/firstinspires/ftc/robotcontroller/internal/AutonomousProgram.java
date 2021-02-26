package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;

/**
 This is a test op mode for season 2020/2021: Ultimate Goal
 Created on September 13th 2020
 */

@Autonomous(name="Autonomous", group="Autonomous")
public class AutonomousProgram extends RobotFunctions
{
    int captureCounter = 0;

    private ElapsedTime runtime = new ElapsedTime();

    private DriveBaseData driveBaseData = null;
    private DcMotor wobbleMotor;
    private ServoData wobbleServo;

    private WebcamData webcamData = null;


    private IMUData imuData;
    private DigitalChannel touchSensor;

    private NormalizedColorSensor colourSensor;


    private int square = 0;
    private int aPosition, bPosition, cPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing, do not press play");
        telemetry.update();

        driveBaseData = new DriveBaseData("LeftFront","RightFront","LeftBack", "RightBack", 75, 1440, hardwareMap);

        driveBaseData.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        colourSensor = hardwareMap.get(NormalizedColorSensor.class, "ColourSensor");

        wobbleServo = new ServoData("WobbleServoArm", 0.0, 0.5, hardwareMap, Servo.Direction.FORWARD);


        imuData = new IMUData("imu", hardwareMap);

        webcamData = new WebcamData(hardwareMap, telemetry);


        sleep(3000);

        //Bitmap pic = webcamData.GetImage();
        //saveBitmap(pic);
        Bitmap picture = webcamData.GetImage();
        saveBitmap(picture);

        /*picture = webcamData.GetImage();
        saveBitmap(picture);
        picture = webcamData.GetImage();
        saveBitmap(picture);*/

        //webcamData.CloseCamera();

        int counter = 0;


        for (int x = 0; x < picture.getWidth(); x+=5) {
            for (int y = 0; y < picture.getHeight() ; y+=5) {
                float[] HSV = new float[3];
                Color.colorToHSV(picture.getPixel(x, y), HSV);

               // if(HSV[2] != 0)
                   // counter++;

                if(HSV[0] < 40)
                {
                    counter++;
                }
            }
        }

        if(picture == null)
            telemetry.addData("Status", "Failure: Webcam was not able to get an image, try reinitializing");
        else
            telemetry.addData("Status", "Initialized, you can now press play");

        telemetry.addLine("Counter: " + counter);
        telemetry.addLine("Width: " + picture.getWidth());
        telemetry.addLine("Height: " + picture.getHeight());


        telemetry.update();

        waitForStart();

       // Bitmap pic = webcamData.GetImage();
       // saveBitmap(pic);

        runtime.reset();


        telemetry.update();
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
}