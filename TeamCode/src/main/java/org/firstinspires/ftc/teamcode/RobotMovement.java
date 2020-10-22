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

    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;

    @Override
    public void init(){
        HardwareMap hardwareMap = null;
        leftBack = hardwareMap.dcMotor.get("");
        leftFront = hardwareMap.dcMotor.get("");
        rightBack = hardwareMap.dcMotor.get("");
        rightFront = hardwareMap.dcMotor.get("");
        
    }

    @Override
    public void loop() {
        leftBack.setPower(0.7);
        leftFront.setPower(0.7);
        leftBack.setPower(0.7);
        rightFront.setPower(0.7);
    }
}
