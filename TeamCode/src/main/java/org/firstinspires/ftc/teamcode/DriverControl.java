package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Driver Control", group="TeleOp")
public class DriverControl extends RobotFunctions
{
    private ElapsedTime runtime = new ElapsedTime();

    private DriveBaseData driveBaseData = null;

    private DcMotorEx intakeMotor = null;
    private DcMotorEx shooterMotor = null;

    private ServoData wobbleServo;
    private ServoData ringServoArm;

    private boolean wobbleDown;
    private boolean ringFlick;

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        driveBaseData = new DriveBaseData("LeftFront","RightFront","LeftBack", "RightBack", 75, 1440, hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");

        wobbleServo = new ServoData("WobbleServoArm", 0.0, 0.5, hardwareMap, Servo.Direction.FORWARD);
        ringServoArm = new ServoData("RingServoArm", 0.0, 1.0, hardwareMap, Servo.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        double leftY = gamepad1.left_stick_y; //driving
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x; //turning

        double leftBackPower = Range.clip(leftY - leftX + rightX, -1.0, 1.0);
        double leftFrontPower = Range.clip(leftY + leftX + rightX, -1.0, 1.0);
        double rightBackPower = Range.clip(leftY + leftX - rightX, -1.0, 1.0);
        double rightFrontPower = Range.clip(leftY - leftX - rightX, -1.0, 1.0);

        driveBaseData.SetPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        int shooterPower = Range.clip((int)gamepad1.right_trigger - (int)gamepad1.left_trigger, -1, 1);

        shooterMotor.setPower(shooterPower);

        int intakePower = 0;

        if(gamepad1.right_bumper)
            intakePower = 1;
        else if (gamepad1.left_bumper)
            intakePower = -1;

        intakeMotor.setPower(intakePower);

        if(gamepad1.left_trigger > 0)
            ringFlick = !ringFlick;

        if(ringFlick)
            SetServoPosition(ringServoArm.servo, ringServoArm.targetPosition);
        else
            SetServoPosition(ringServoArm.servo, ringServoArm.startPosition);

        if(gamepad1.a)
            wobbleDown = !wobbleDown;

        if(wobbleDown)
            SetServoPosition(wobbleServo.servo, wobbleServo.targetPosition);
        else
            SetServoPosition(wobbleServo.servo, wobbleServo.startPosition);



        telemetry.addLine(runtime.toString());


        telemetry.addData("Runtime: ", runtime.time());
        telemetry.update();

        //Drive forward
    }
}