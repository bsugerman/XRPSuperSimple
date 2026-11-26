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

Guess what? That is what Feedforward motor controls do. If you've
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
sensor and stopping at 20 cm from an object. You start 0.5 m away, 
we define our **setpoint** as 0.2 m, and we define `kP=3`. 

* Reading 1: You are at 0.5 m and the goal is 0.2 m.
  * The error is `0.5 - 0.2 = 0.3 m`
  * `kP` calculates `0.3*3` and sends 90% power. The robot rockets forward
* Reading 2: You are now at 0.3 m.
  * The error is `0.3 - 0.2 = 0.1 m`.
  * `kP` calculates `0.1*3` and sends 30% power. The robot slows down, but
  it was already moving fast, so its momentum still carries it fast.
* Reading 3: Momentum made it overshoot and it is now at 0.1 m.
  * The error is `0.1 - 0.2 = -0.1 m`
  * `kP` calculates `-0.1*3` and sends 30% power to drive the robot
  backwards.
* Reading 4: The robot overshoots to 0.25 m.
  * The error is `0.25 - 0.2 = 0.05 m`
  * `kP` calculates 15% power to drive forward.
* Future readings: 
  * The robot drives forward, and overshoots, backs up and overshoots,
  and keeps this up until the power level is too low to overcome static
  friction. The motors whine and never stop. 

OK, these oscillations are ridiculous, and unnecessary. Let's build in
a **tolerance** that says "if we are within this distance from our
setpoint, stop." We could use a tolerance of 1 cm. That stops some of
the oscillation at the end. But the speed of our first jump from
50 to 30 cm was clearly way too fast. We could lower `kP` and the robot would
move slower in that first jump. It would also move slower in every
other step. So instead, let's try to slow it down when it tries to
change its speed by a huge amount by applying a `kD` (Derivative)
term, to stop huge sudden changes. So let's imagine we have `kP` and
`kD` tuned nicely, so the robot doesn't violently overshoot anymore.
Maybe `kP` is now 2. Let's retrace the readings for the steps above:

* Reading 1: You are at 0.5 m and the goal is 0.2 m. `kP` sends 60% power and
  the robot drives forward to 0.35 m.
* Reading 2: You are now 0.15 m away.  `kP` sends 30% power and drives
  the robot forward to 0.28 m.
* Reading 3: You are now 0.08 m away. `kP` sends 16% power and drives
 the robot forward to 0.23 m.
* Reading 4: You are now 0.03 m away. `kP` sends 6% power, which is too little
  to move the robot, and the motors whine forever because they aren't within
  our 0.01-m tolerance.

The `kI` (Integral) term is a "frustration timer" that adds up your
remaining error over time to give the motors an extra push when `kP`
alone isn't strong enough to reach the target.  Continuing directly
with the example above, here is how `kI` solves it.

* Reading 5: You are still 0.03 m away. `kI` sees an 0.03-m error and adds 1%
  extra power.
* Reading 6-10: You are still 0.03 m away. 'kI` adds another 1% extra power
  at each reading.
* Reading 11: You reach 12% power which was enough to overcome friction,
  and the robot creeps forward until it reaches less than 0.01-m distance. 

There is a danger with using `kI` however. By the time we reach the threshold
distance, `kI` might have added in so much extra power that we still
overshoot. This is called "the integral windup". The Integral term can
be very helpful when fighting against a constant force like gravity or
to keep a flywheel spinning at an exact speed, but for driving a robot,
people often leave `kI=0`.

At this point, you might be saying to yourself "Wait a minute!
Feed-forward provides a static-friction term `S` that keeps the motor
able to turn all the time." And indeed, feed-forward can be added to
PID for this very purpose. We set a minimum power level that is _just_
enough to almost overcome static friction, and then if the `kP` term
gives 2% power, that is 2% on top of whatever was needed to overcome
internal friction, and so to motors actually turn. No need for `kI`.

## More about FeedForward 

Feed Forward also includes a `kV` term for velocity and `kA` term for
acceleration, in addition to the `kS` term for static friction. In our
example above, if we needed 11% power just to make the motor start to turn,
we would set `kS=0.11`. Once we reach the setpoint, we cut all power to
0% so kS stops pushing.

`kV` and `kA` are only used when there is a moving target, meaning
we aren't saying "get to 0.2-m distance eventually," but we are
following a smooth speed schedule that dictates how fast you
should be moving at every millisecond. These are called "Motion Profiles"
and a common one is the Trapezoidal Profile, which has a ramp up
to a constant speed, a duration at that speed, then a ramp down to
stopping. 

* `kV` is velocity feedforward: the poewr to keep cruising at steady
 speed.  Imagine you want the robot to drive at a steady 1 meter per
 second. Because of air resistance, gear drag, and motor back-EMF
 (electricity the spinning motor generates against your battery), it
 might take a constant 40% power just to maintain 1 m/s. If you only
 used kP, the robot would have to fall behind its schedule to create
 enough error to generate that 40% power. kV supplies that 40% upfront
 so kP stays near zero.

* `kA` is acceleration feedforward: the extra burst of power needed to
change speed. Pushing a heavy robot from 0 m/s up to 1 m/s takes way
more muscle than keeping it rolling once it's already moving (just
like pushing a heavy shopping cart). When your motion profile says
"accelerate right now," kA gives an instant kick of extra power so the
heavy robot doesn't lag behind at the start-and gives a reverse kick
when it's time to brake.

## When do I need what?

* Simple point-to-point movement: `kS` overcomes static friction, and `kP`
gets you from Point A to B. You might need `kD` for braking.

* Smoothly-controlled speed or autonomous path: `kS`, `kV`, `kA` are
going to do 90% of the work, and `kP` will clean up little bumps or
wiggles, like literal bumps in the road or a battery lag.

# Using feedforward with our XRP robot

WPILib includes built-in support for PID and feedforward controlling.
Unfortunately, they give output in Volts and our XRP robot needs drive
values in percentages from -1 to 1. So using WPILib's
`SimpleMotorFeedforward` class would require a little swindle by dividing
all its output by 12 (assuming it thinks it is sending 12 V to a motor). We'll
try that out in a bit, but first, let's try some cheap-and-dirty methods.

** Adding feedforard `kS` manually.

We have a controller sending drive values between -1 to +1 to the
drivetrain in `robot.py`'s `robotPeriodic` function:

```
    def robotPeriodic(self) -> None:
        """This function is called periodically (by default, 50 times per second)"""
        super().robotPeriodic()

        # 1. driving: take the speeds from joystick
        fwd_speed = wpimath.applyDeadband(
            -self.joystick.getRawAxis(1),
            DEADBAND
        )
        turn_speed = wpimath.applyDeadband(
            self.joystick.getRawAxis(0),
            DEADBAND
        )

        self.drivetrain.arcadeDrive(fwd_speed, turn_speed)
        self.drivetrain.periodic()  # updates odometry
```

If we want to implement a feedforward static friction term, we just need to
make sure that the two speeds `fwd_speed` and `turn_speed` are never less
than some `kS` value that we have to determine by experimentation. Right now
the speed value runs from 0.0 to 1.0. We need to remap that to run from
`kS` to 1.0, except if speed==0.0, in which case we leave it alone. We
also need to make sure that if speed is negative, it stays that way. There
are lots of ways to compute this. Here is just one:

```
def apply_ks(speed: float, ks: float = kS) -> float:
    if speed == 0.0:
        return 0.0
    return math.copysign(ks + (1.0 - ks) * abs(speed), speed)
```

Now when we read a speed from the controller, after applying deadband,
we can also apply feedforward correction. Or we can be really clever and
do both deadband and feedforward in the same function.

