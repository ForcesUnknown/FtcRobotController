package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Vector;

/**
 This is a test op mode for season 2020/2021: Ultimate Goal
 Created on September 13th 2020
 */

@Autonomous(name="TestAutonomous", group="Autonomous")
public class TestAutonomousProgram extends RobotFunctions
{
    private ElapsedTime runtime = new ElapsedTime();

    private DriveBaseData driveBaseData = null;
    private ServoData measureServo;
    private ServoData wobbleServo;


    private IMUData imuData;
    private TouchSensor touchSensor;

    private NormalizedColorSensor colourSensor;


    private int square;
    private int aPosition, bPosition, cPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        driveBaseData = new DriveBaseData("LeftFront","RightFront","LeftBack", "RightBack", 75, 1440, hardwareMap);

        driveBaseData.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        colourSensor = hardwareMap.get(NormalizedColorSensor.class, "ColourSensor");

        wobbleServo = new ServoData("WobbleServoArm", 0.6, 0.3, hardwareMap);
        measureServo = new ServoData("TouchServoArm", 1.0, 1.0, hardwareMap);
        touchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");

        imuData = new IMUData("imu", hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        SetServoPosition(wobbleServo.servo, wobbleServo.targetPosition);

        sleep(100);

        //Drive forward
        DriveFrontBackDistance(driveBaseData, 1, 765, 5);
      /*  VectorData[] range = new VectorData[2];
        range[0] = new VectorData(0, 0, 0);
        range[0] = new VectorData(255, 255, 100);

        DriveLeftRightColour(driveBaseData, 1, range, colourSensor, 100);*/
        double servoPosition = measureServo.startPosition;
        while(!touchSensor.isPressed())
        {
            servoPosition -= (measureServo.targetPosition - measureServo.startPosition) / 15;
            SetServoPosition(measureServo.servo, servoPosition);
            sleep(30);
        }
        if (Range(measureServo.servo.getPosition(), aPosition, 0.15))
            square = 0;
        else if(Range(measureServo.servo.getPosition(), bPosition, 0.15))
            square = 1;
        else
            square = 2;

        switch(square)
        {
            case(0):
                DriveFrontBackDistance(driveBaseData, 1, 915, 5);
                break;
            case(1):
                DriveFrontBackDistance(driveBaseData, 1, 1520, 5);
                DriveLeftRightDistance(driveBaseData, 1, 600, 5);
                break;
            case(2):
                DriveFrontBackDistance(driveBaseData, 1, 2130, 5);
                break;
        }

        sleep(100);

        SetServoPosition(wobbleServo.servo, wobbleServo.startPosition);

        sleep(100);

        telemetry.addLine("Square = " + square);
        telemetry.update();
    }
}
