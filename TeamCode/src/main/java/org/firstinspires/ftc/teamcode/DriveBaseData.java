package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveBaseData {

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    public double wheelCircumference;
    public double ticksPerCentimeter;
    public int encoderTicksPerRotation; // I believe its is 28 for rev and 1440 for Tetrix but i could be wrong

    public DriveBaseData(String leftFront, String rightFront, String leftBack, String rightBack, double wheelDiameter, int encoderTicks, HardwareMap hardwareMap)
    {
        //Hello
        this.encoderTicksPerRotation = encoderTicks;
        this.wheelCircumference = wheelDiameter * Math.PI;
        ticksPerCentimeter = encoderTicksPerRotation / wheelCircumference;

        this.leftFront = hardwareMap.get(DcMotor.class, leftFront);
        this.rightFront = hardwareMap.get(DcMotor.class, rightFront);
        this.leftBack = hardwareMap.get(DcMotor.class, leftBack);
        this.rightBack = hardwareMap.get(DcMotor.class, rightBack);

        this.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void SetPower(double power)
    {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void SetPower(double powerLF, double powerRF, double powerLB, double powerRB)
    {
        leftFront.setPower(powerLF);
        rightFront.setPower(powerRF);
        leftBack.setPower(powerLB);
        rightBack.setPower(powerRB);
    }

    public void SetMode(DcMotor.RunMode runMode)
    {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    public void SetTargetPosition(int position)
    {
        leftFront.setTargetPosition(position);
        rightFront.setTargetPosition(position);
        leftBack.setTargetPosition(position);
        rightBack.setTargetPosition(position);
    }

    public void SetTargetPosition(int positionLF, int positionRF, int positionLB, int positionRB)
    {
        leftFront.setTargetPosition(positionLF);
        rightFront.setTargetPosition(positionRF);
        leftBack.setTargetPosition(positionLB);
        rightBack.setTargetPosition(positionRB);
    }
}
