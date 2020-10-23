package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
@Disabled
public class RobotMovement extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;

    @Override
    public void init(){
        HardwareMap hardwareMap = null;
        leftBack = hardwareMap.dcMotor.get("leftBack_wheel");
        leftFront = hardwareMap.dcMotor.get("leftFront_wheel");
        rightBack = hardwareMap.dcMotor.get("rightBack_wheel");
        rightFront = hardwareMap.dcMotor.get("rightFront_wheel");
        
    }

    @Override
    public void loop() {
        double leftBackPower;
        double leftFrontPower;
        double rightBackPower;
        double rightFrontPower;

        double lefty = gamepad1.left_stick_y; //driving
        double leftx = gamepad1.left_stick_x;
        double rightx  =  gamepad1.right_stick_x; //turning
        leftBackPower = Range.clip(lefty - leftx + rightx, -1.0, 1.0); //-1+-1=-1
        leftFrontPower = Range.clip(lefty + leftx + rightx, -1.0, 1.0); //(-1) - (-1) = 0
        rightBackPower = Range.clip(lefty + leftx - rightx, -1.0, 1.0); // -1+-1=-1
        rightFrontPower = Range.clip(lefty - leftx - rightx, -1.0, 1.0); //-1--1=0

        leftBack.setPower(leftBackPower);
        leftFront.setPower(leftFrontPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

    }
}