# Super simple Python code to get XRP robot working
With joystick and odometry.

This code goes with the video https://youtu.be/doHw68EDf84?si=42V_fk2upLhxCt7r

# Things to try:

## Joystick controls

The joystick may show on the simulator that it never actually
reaches `0.0`, so this is where a deadband comes in. In this code, the
drive is defined in `drivetrain.py` under the class `Drivetrain`. The
actual drive is defined as `self.drive = DifferentialDrive()` and
`DifferentialDrive` is a child class of the parent
`RobotDriveBase`. Ths parent class automatically comes with a deadband
of `0.02`.  You can increase this by calling
`self.drive.setDeadband(NewValue)` to apply that to all drive inputs. Easy!
Or you can challenge yourself further by using the build in command
`wpimath.applyDeadband()` on the `fwd_speed` and `turn_speed` variables
in `robot.py`.

Another juicy little gem is that command `arcadeDrive`
in the `Drivetrain` class is just a convenience call to `self.drive.arcadeDrive`, another build-in child class of WPILib's `RobotDriveBase`. The authors
included a 3rd option to this function:

```def arcadeDrive(xSpeed, zRotation, squareInputs=True)```

That boolean at the end uses the square of your speed and rotation values,
to essentially decrease the input sensitivity at low speeds. That means
if your joystick reads 0.1, the drive is receiving 0.01. So you can try
turning this off by changing the function that calls `arcadeDrive` to
set `squareInputs=False`.

## Distance sensor:

Wouldn't it be nice to use the distance sensor to stop a collision before
it happens? You can with a very simple addition to the code.
Everything in `robotPeriodic` is called 50 times per second, so why
not check whether there is anything within a threshold distance of the
sensor, and stop the robot if there is? The drivetrain has a function
`getDistanceToObstacle()` that returns the sensor distance in meters.
If you look at the code, you'll see that it has been written to return
`math.nan` if the distance is larger than 0.5 m. You can change that if you
like.

In the `robotPeriodic` you can put a line that says "if the value of
`getDistanceToObstacle()` is less than X, stop the robot" with the following:

```
    if self.drivetrain.getDistanceToObstacle < 0.2 and fwd_speed > 0:
        fwd_speed = 0.
```

Note that we only want to stop the robot from moving **forward** but
not stop it altogether, otherwise it could never back up! Also note,
setting the threshold value of 0.2 m here is bad coding if you are
trying to make a huge system that lots of people can work on. It is
never a good idea to hardcode values that someone might want to change
(same with the deadband). It's a much better idea to define constants
in a special constants.py file that is imported, or at a minimum, put
the values at the top of your .py file (after the imports) and then
use the variable names later. Many people write constants in all-caps
when they have been defined at the top of the file, as a clue
to the reader to go look there to change it. So you could edit the `robot.py`
file so the top reads

```
...
os.environ["HALSIMXRP_PORT"] = "3540"

DEADBAND = 0.1

class MyRobot(wpilib.TimedRobot):
...
```

# My observations:

## This drive sucks!!

If you only push a little on the joystick, the motors make a high-pitch
whine and they don't do anything. You have to push a decent amount on the
controller before they do anything, and then they leap into action. It's
especially bad for turning. What can be done?

The whining is because the motors are using pulse-width modulation
(PWM) and you are literally hearing the pulses resonate through the
motors.  In a very general sense, you should be able to change this
frequency to be outside the range of human hearing. Unfortunately, the
actual PWM signal sent to the onboard H-bridge motor
drivers (DRV8411A on V1 boards) is generated
directly by the XRP's onboard microcontroller firmware and not by the
WPILib software or another user-controlled system. So, either you
download, modify, and flash the `xrp-wpilib-firmware` in C++ onto the
Raspberry Pi, or you deal with the noise.

But let's go back to **why** this noise is happening. The motor isn't
able to overcome all its internal sources of friction (static friction)
and so you are hearing the pulses shudder through the motor, plastic housing,
robot, etc. Why not just adjust everything so that the smallest
speed the motor is allowed to turn is just above this level of static
friction?

Guess what? That is what Feed-Forward motor controls do. If you've
ever heard someone talk about "PID Tuning" and they mention setting
the "S, V, and A" constants, they are doing feed-forward motor
control. The "S" term (sometime written `kS` in WPILib code) is the
amount of oomph (power, voltage, current, whatever) that is needed to
overcome static friction.

## What is PID and why aren't we using it here?

Think of PID as a feedback loop. You put in a little bit of input (move the
joystick) and expect a certain outcome (robot moves). If it doesn't move if
pushed a little, then you keep pushing more until it moves. But then it jumps
forward really quickly and you didn't want that, so you back off on the
controller, and fiddle around back and forth quickly until you found the
speed you wanted. You just did PD control on-the-fly with your brain. The robot
moves **proportionally** to how much you push forward on the controller
(the **P** term). But when it moved too fast (a large change in a short time)
you pushed a little less -- that change is a **derivative** and you subtracted
signal when the derivative was too high (the **D** term). Now let's say you
drove it forward for a while but noticed it keeps turning a little to
the right. You say to yourself "I need to push the controller up and
slightly to the left to make it go straight." You noticed that over a long
time, it wasn't behaving quite correctly and you took the sum (or **integral**)
of that behavior and corrected for it. You just did PD control on-the-fly.
Put them all together and that is PID.

In code, PID is a math equation for automation, where you define the
constants `kP`, `kI`, and `kD`, and a goal, like move forward 1 meter
or rotate this arm to 90 degrees. Let's use the example of the distance
sensor and stopping at 20 cm from an object. You start 50 cm away and
we define our **setpoint** as 20 cm. 

1. You need to move 50-20=30 cm. The `kP` term says to move forward
 proportional to that amount, meaning fast.
2. The next distance reading says you are 30 cm away, so you need to move 10
 cm. Your 'kP' term now sees a smaller distance and will say to move
 proportional to 10 cm, meaning slower.
3. The next distance reading says you are 15 cm away. You wanted to be 20 cm.
 So you need to move 15-20=-5 cm, backwards! The `kP` term sees a negative
 and needs to move proporionally to that, meaning backwards slowly.
4. The next distance reading says you are 22 cm away. You wanted to be 20 cm.
 So you need to move 22-2=2 cm forwards. The `kP` term will now move you
 forward, and you probably overshoot.
5. Now you oscillate backwards and forwards over and over again, trying
 to get infinitely close to exactly 20 cm.

OK that's stupid. Let's build in a **tolerance** that says "if we are
within this distance from our setpoint, stop." We could use a
tolerance of 2 cm. That stops some of the oscillation. But maybe the
speed of our first jump from 50 to 30 cm was way too fast. We could
lower `kP` and the robot would move slower in that first jump. It
would also move slower in every other step. So instead, let's try to
slow it down when it tries to change its speed by a huge amount by
applying a `kD` term, to stop huge sudden changes. 
