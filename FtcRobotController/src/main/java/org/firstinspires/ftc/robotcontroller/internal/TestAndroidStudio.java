package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestAndroidStudio", group="Autonomous")
public class TestAndroidStudio extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();

        sleep(1000);


        telemetry.addLine("Doing Stuff");
        telemetry.update();

        sleep(10000);

    }
}
