package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoData {

    public Servo servo;
    public double startPosition;
    public double targetPosition;

    public ServoData(Servo servo, double startPosition, double targetPosition)
    {
        this.servo = servo;
        this.startPosition = startPosition;
        this.targetPosition = targetPosition;

        servo.setPosition(startPosition);
    }
}
