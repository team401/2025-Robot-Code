# LED Guide
Guide for how LEDs work / how we use LEDs :D
## Colors

List of Colors:
``lockedOnHang  =  Color.kGreen;``
`` holdingAlgae  =  Color.kYellow;``
``holdingCoral  =  Color.kMagenta;``
 ``targetOnReef  =  Color.kBlue;``
``targetOnReefL1  =  Color.kCyan;``
``targetOnReefL2  =  Color.kTeal;``
``targetOnReefL3  =  Color.kPurple;``
``targetOnReefL4  =  Color.kMagenta;``
``targetOnProcessor  =  Color.kYellow;``
``targetOnNet  =  Color.kWhite;``
``targetOnCoral  =  Color.kLime;``
``off  =  Color.kBlack;``
Pretty simple to use. You make a Color and then give it a pre-made color or use RGB to make your own.
Ex:
``public static final Color red = new Color(255,0,0);``
``public static final Color red = Color.kRed;``



>All Colors are defined in [``LEDConstants``](#our-led-subsystem)
## LED Patterns
LEDPatterns are where things get interesting because this is where the magic happens. When you make an LED do something it is going to be through a LEDPattern. There are a lot of things that they can do so if you want to read up on the go to the [WPILib Documentation](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html) & [Github API Pull request](https://github.com/wpilibsuite/allwpilib/pull/6344). 

The way we use them is to split each AddressableLEDBufferView into thirds using ``.steps(Map.of(0,  (Insert Color Here),  1  /  3.0,  (Insert Color Here)));`` This splits the view into thrids and applies a color to the first third, and then another color for the rest of the section.
> **Note:** **This is how we run every LEDPattern** except rainbow.

## Our LED Subsystem


### LED Constants.java

Pretty straight forward constants file. It holds data like the total length, the lengths for each buffer view. As well as different miscellaneous constants like ``rainbowSpeed``

### LED.java

This is where we do everything for making the LEDs run. The few main parts are: ``periodic()``, ``addPattern``,``runPattern``, and ``applyPatterns``



**addPattern/addSplitPattern**
	 adds the LEDPattern to an ArrayList to be applied with ``applyPatterns()``
**runPattern/runSplitPattern**
	rather than adding a LEDPattern to a list it just directly applies the pattern to the LEDs
**applyPatterns**
	uses  ``LEDPattern.overlayOn();``  to add LEDPatterns from the ArrayList and checks adds them to the LEDPattern ``leftFinal``, ``rightFinal`` respectfully. 
> **Note:** When you run this in periodic() it will implicitly be creating a priority order of which LEDPatterns to run based off of which LEDPattern comes last on the list.
>
>Ex.
```java @Override

public  void  periodic()  {
	addSplitPattern(Color.kBlack,Color,kGreen);
	addPattern(Color.kRed);
	applyPatterns();
}
```
>Only the ``Color.kRed`` would be displayed. This also goes for parts that are using steps or maps.

**periodic()**
When you look at ``periodic()`` it will have a bunch of if-else statements these are used to determine what is happening on the robot at that time.
The first if statement simply checks if the robot is disabled/disconnected. If it is then it will use the ``runPattern();`` to display a rainbow animation. When it gets enabled it runs a list of if statements and then adds the patterns to the [applyPatterns](#applypattern) ArrayList. 
Ex.
```java
if  (scoringSubsystem !=  null  &&  scoringSubsystem.getGamePiece()  ==  GamePiece.Algae)  {

addSplitPattern(holdingAlgae, clear);

}
```
This checks first for the fact that the scoring subsystem is real, then it gets the game piece from it. If that game piece is an Algae it adds a split pattern to the list, if not it moves onto the other statements. If you wanted to know what each pattern actually was then you would look at the LEDPattern
Ex.
```java
public  LEDPattern  holdingAlgae  = LEDPattern.steps(Map.of(0,  LEDConstants.holdingAlgae,  1  /  3.0, LEDConstants.off));
```
Runs the holdingAlgae color for the top third of the left buffer view and then the right buffer view is clear.

This is a list of what if statement does:
1. Check for Algae
2. Check for Coral
3. Check for FieldTarget L1 (elevator algae intake height/ score height)
4. Check for FieldTarget L2
5. Check for FieldTarget L3
6. Check for FieldTarget L4
7. Checks if we are locked onto cage
8. Checks if the OTF position is around the Reef


## Usages
We use this in our own ``initSubsystems.java`` 
```java
public static LED initLEDs(
      ScoringSubsystem scoringSubsystem, ClimbSubsystem climbSubsystem, Drive drive) {

    switch (ModeConstants.currentMode) {
      case REAL:
        return new LED(scoringSubsystem, climbSubsystem, drive);
      case SIM:
      case MAPLESIM:
        return new LED(scoringSubsystem, climbSubsystem, drive);

      case REPLAY:
        throw new UnsupportedOperationException("LED replay is not yet implemented.");
      default:
        throw new UnsupportedOperationException(
            "Non-exhaustive list of mode types supported in InitSubsystems (got "
                + ModeConstants.currentMode
                + ")");
    }
  }
```
Then in ``robotContainer.java`` we use it:
```java
if  (FeatureFlags.synced.getObject().runLEDs)  {

led =  InitSubsystems.initLEDs(scoringSubsystem, climbSubsystem, drive);

}
```
This should clear up some confusion on how we run our LED subsystem.