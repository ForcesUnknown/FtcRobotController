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

        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        leftFront = hardwareMap.get(DcMotor.class,"LeftFront");
        rightBack = hardwareMap.get(DcMotor.class,"RightBack");
        rightFront = hardwareMap.get(DcMotor.class,"RightFront");
        
    }

    @Override
    public void loop() {
        double leftBackPower;
        double leftFrontPower;
        double rightBackPower;
        double rightFrontPower;

        double leftY = gamepad1.left_stick_y; //driving
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x; //turning
        
        leftBackPower = Range.clip(leftY - leftX + rightX, -1.0, 1.0); //-1+-1=-1
        leftFrontPower = Range.clip(leftY + leftX + rightX, -1.0, 1.0); //(-1) - (-1) = 0
        rightBackPower = Range.clip(leftY + leftX - rightX, -1.0, 1.0); // -1+-1=-1
        rightFrontPower = Range.clip(leftY - leftX - rightX, -1.0, 1.0); //-1--1=0

        leftBack.setPower(leftBackPower);
        leftFront.setPower(leftFrontPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        telemetry.addLine(runtime.toString());
        telemetry.update();

    }
}