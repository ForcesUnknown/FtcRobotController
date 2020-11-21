package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 This is a test op mode for season 2020/2021: Ultimate Goal
 Created on September 13th 2020
 */

@TeleOp(name="TestAutonomous", group="TestPrograms")
public class TestAutonomousProgram extends RobotFunctions
{
    private ElapsedTime runtime = new ElapsedTime();

    private DriveBaseData driveBaseData = null;
    private ServoData testServo;

    private IMUData imuData;
    private TouchSensor touchSensor;

    private int square;
    private int aPosition, bPosition, cPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        driveBaseData = new DriveBaseData("LeftFront","RightFront","LeftBack", "RightBack", 75, 1440, hardwareMap);

        //testServo = new ServoData("TouchServoArm", 0.0, 1.0, hardwareMap);

        imuData = new IMUData("imu", hardwareMap);
        //touchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //Drive forward 10 cm at power 1, if it takes more than 5 seconds stop
        DriveFrontBackDistance(driveBaseData, 1, 10, 5);
        /*double servoPosition = testServo.startPosition;
        while(!touchSensor.isPressed())
        {
            servoPosition = Lerp(servoPosition, testServo.targetPosition);
            SetServoPosition(testServo.servo, servoPosition);
            sleep(10);
        }
        if (Range(testServo.servo.getPosition(), aPosition, 0.15))
            square = 0;
        else if(Range(testServo.servo.getPosition(), bPosition, 0.15))
            square = 1;
        else
            square = 2;*/


    }
}
