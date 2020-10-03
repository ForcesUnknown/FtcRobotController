package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        driveBaseData = new DriveBaseData(hardwareMap.get(DcMotor.class, "LeftFront"), hardwareMap.get(DcMotor.class, "RightFront"),
                hardwareMap.get(DcMotor.class, "LeftBack"), hardwareMap.get(DcMotor.class, "RightBack"), 50, false);

        testServo = new ServoData(hardwareMap.get(Servo.class, "ServoA"), 0.0, 1.0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();



    }
}
