package org.firstinspires.ftc.robotcontroller.internal;

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
    private DcMotor wobbleMotor = null;


    private ServoData wobbleServo;
    private ServoData ringServoArm;

    private boolean wobbleDown;
    private boolean wobbleGrab;
    private boolean ringFlick;

    private double lastRingFlick;
    private double ringFlickTime = 1500; // -> change this value (in ms) to change how quickly you can load the shooter.

    //change these for dcmotor position
    private final int wobbleArmUp = 0;
    private final int wobbleArmDown = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        driveBaseData = new DriveBaseData("LeftFront","RightFront","LeftBack", "RightBack", 75, 1440, hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        wobbleMotor = hardwareMap.get(DcMotor.class, "WobbleMotor");

        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setPower(0.5f);
        wobbleMotor.setTargetPosition(wobbleArmUp);

        wobbleServo = new ServoData("WobbleServoArm", 0.0, 0.5, hardwareMap, Servo.Direction.FORWARD);
        ringServoArm = new ServoData("RingServoArm", 0.0, 0.2, hardwareMap, Servo.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while(opModeIsActive()) {
            //Driving ----------------------------------------------------------------------------------

            double leftY = -gamepad1.left_stick_y; //driving
            double leftX = gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x; //turning

            double leftBackPower = Range.clip(leftY - leftX + rightX, -1.0, 1.0);
            double leftFrontPower = Range.clip(leftY + leftX + rightX, -1.0, 1.0);
            double rightBackPower = Range.clip(leftY + leftX - rightX, -1.0, 1.0);
            double rightFrontPower = Range.clip(leftY - leftX - rightX, -1.0, 1.0);

            driveBaseData.SetPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            //Shooter/Intake------------------------------------------------------------------------

            double shooterPower = Range.clip(gamepad1.right_trigger, 0, 0.66);

            shooterMotor.setPower(shooterPower);

            int intakePower = 0;

            if (gamepad1.right_bumper)
                intakePower = 1;
            else if (gamepad1.left_bumper)
                intakePower = -1;

            intakeMotor.setPower(intakePower);

            //Ring Flick --------------------------------------------------------------------------------
            if (gamepad1.left_trigger > 0 && lastRingFlick + ringFlickTime < runtime.milliseconds())
            {
                lastRingFlick = runtime.milliseconds();
                ringFlick = true;
            }

            if(lastRingFlick + (ringFlickTime / 2) < runtime.milliseconds())
                ringFlick = false;
            if (ringFlick)
                SetServoPosition(ringServoArm.servo, ringServoArm.targetPosition);
            else
                SetServoPosition(ringServoArm.servo, ringServoArm.startPosition);

            //Wobble ------------------------------------------------------------------
            if (gamepad1.a)
                wobbleDown = !wobbleDown;

            if(gamepad1.b)
                wobbleGrab = !wobbleGrab;

            if (wobbleDown)
                wobbleMotor.setTargetPosition(wobbleArmDown);
            else
                wobbleMotor.setTargetPosition(wobbleArmUp);

            if (wobbleGrab)
                SetServoPosition(wobbleServo.servo, wobbleServo.targetPosition);
            else
                SetServoPosition(wobbleServo.servo, wobbleServo.startPosition);


            telemetry.addLine(runtime.toString());


            telemetry.addData("Runtime: ", runtime.time());
            telemetry.update();
        }

        //Drive forward
    }
}
