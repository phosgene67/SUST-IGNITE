#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H
class MotorDriver {
public:
    void begin();

    void drive(int leftSpeed, int rightSpeed);
    void forward(int speed);
    void backward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void brake();
    void stop();

    void setLeftMotor(int speed);
    void setRightMotor(int speed);

    void testForward();
    void testBackward();
    void testLeft();
    void testRight();

    int getLastLeftSpeed()  const { return _lastLeftSpeed; }
    int getLastRightSpeed() const { return _lastRightSpeed; }

private:
    static const int MAX_SPEED  = 255;
    static const int TEST_SPEED = 150;
    static const unsigned long TEST_DURATION_MS = 1000;

    int _lastLeftSpeed  = 0;
    int _lastRightSpeed = 0;

    int _clampSpeed(int speed) const;
};

#endif
