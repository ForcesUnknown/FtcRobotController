package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DriverControl", group="TeleOp")
public class RobotMovement extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;

    private DcMotor intakeMotor;
    private DcMotor shooterMotor;

    @Override
    public void init()
    {
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        leftFront = hardwareMap.get(DcMotor.class,"LeftFront");
        rightBack = hardwareMap.get(DcMotor.class,"RightBack");
        rightFront = hardwareMap.get(DcMotor.class,"RightFront");

        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "ShooterMotor");

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        double leftY = gamepad1.left_stick_y; //driving
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x; //turning

        double leftBackPower = Range.clip(leftY - leftX + rightX, -1.0, 1.0); //-1+-1=-1
        double leftFrontPower = Range.clip(leftY + leftX + rightX, -1.0, 1.0); //(-1) - (-1) = 0
        double rightBackPower = Range.clip(leftY + leftX - rightX, -1.0, 1.0); // -1+-1=-1
        double rightFrontPower = Range.clip(leftY - leftX - rightX, -1.0, 1.0); //-1--1=0

      //  double leftBackPower = Range.clip(leftY - leftX - rightX, -1.0, 1.0); //-1+-1=-1
       // double rightBackPower = Range.clip(leftY + leftX + rightX, -1.0, 1.0); // -1+-1=-1
       // double leftFrontPower = Range.clip(leftY + leftX - rightX, -1.0, 1.0); //(-1) - (-1) = 0
       // double rightFrontPower = Range.clip(leftY - leftX + rightX, -1.0, 1.0); //-1--1=0

       // double leftBackPower = Range.clip(rightX, -1.0, 1.0); //-1+-1=-1
       // double rightBackPower = Range.clip(-rightX, -1.0, 1.0); // -1+-1=-1
       // double leftFrontPower = Range.clip(rightX, -1.0, 1.0); //(-1) - (-1) = 0
       // double rightFrontPower = Range.clip(-rightX, -1.0, 1.0); //-1--1=0



        leftBack.setPower(leftBackPower);
        leftFront.setPower(leftFrontPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        int shooterPower = Range.clip((int)gamepad1.right_trigger - (int)gamepad1.left_trigger, -1, 1);

        shooterMotor.setPower(shooterPower);

        int intakePower = 0;

        if(gamepad1.right_bumper)
            intakePower = 1;
        else if (gamepad1.left_bumper)
            intakePower = -1;

        intakeMotor.setPower(intakePower);


        telemetry.addLine(runtime.toString());

        telemetry.update();

    }
}