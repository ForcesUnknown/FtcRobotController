package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotMovement {
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;

    public void init(){
        HardwareMap hardwareMap;
        leftBack = hardwareMap.dcMotor.get("");
        leftFront = hardwareMap.dcMotor.get("");
        rightBack = hardwareMap.dcMotor.get("");
        rightFront = hardwareMap.dcMotor.get("");
        
    }
}
