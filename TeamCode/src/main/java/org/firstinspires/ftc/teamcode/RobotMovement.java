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

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftBackPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        leftFrontPower = Range.clip(drive - turn, -1.0, 1.0) ;
        rightBackPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightFrontPower = Range.clip(drive - turn, -1.0, 1.0) ;
    }

    public void stop(){

    }
}