package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        driveBaseData = new DriveBaseData("LeftFront","RightFront","LeftBack", "RightBack", 50, 1440, hardwareMap);

        testServo = new ServoData("ServoA", 0.0, 1.0, hardwareMap);

        imuData = new IMUData("IMU", hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //Drive forward 100 cm at power 1, if it takes more than 5 seconds stop
        DriveFrontBackDistance(driveBaseData, 1, 100, 5);


    }
}
