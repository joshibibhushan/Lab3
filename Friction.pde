/**
 **********************************************************************************************************************
 * @file       lab3.pde
 * @author     Bibhushan Raj Joshi
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/



/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/

/* text font */
PFont             f;


/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 30.0;  
float             worldHeight                         = 20.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define maze blocks */
FBox                l1;
FBox                l2;
FBox                l3;
FCircle             c1;
FCircle             c2;
FCircle             c3;
FCircle             c4;
FCircle             c5;
FCircle             c6;
FCircle             c7;
FCircle             c8;
FCircle             c9;
FCircle             c10;
FCircle             c11;
FCircle             c12;
FCircle             c13;
FCircle             c14;
FCircle             c15;
FCircle             c16;
FCircle             c17;

FCircle             b1;
FBox                l4;

//set levels
Boolean level1=Boolean.FALSE;
Boolean level2=Boolean.FALSE;
Boolean level3=Boolean.FALSE;
Boolean level4=Boolean.FALSE;

float forceAngle = 0;
float levelForce = 0;



PVector posEELast = new PVector(0, 0);

String k = "off";

float limit = 0.02;


/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  size(1200, 800);

  /* set font type and size */
  f                   = createFont("Arial", 30, true);

  /* device setup */

  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, "COM3", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);


  widgetOne.device_set_parameters();


  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();


  c1                  = new FCircle(2);
  c1.setPosition(2.5, 14.2);
  c1.setFill(255, 125, 255);
  c1.setDensity(500);
  c1.setSensor(false);
  c1.setNoStroke();
  c1.setStatic(true);

  c2                  = new FCircle(3);
  c2.setPosition(6, 12);
  c2.setFill(255, 255, 0);
  c2.setDensity(500);
  c2.setSensor(false);
  c2.setNoStroke();
  c2.setStatic(true);

  c3                  = new FCircle(2);
  c3.setPosition(12, 10);
  c3.setFill(255, 255, 0);
  c3.setDensity(500);
  c3.setSensor(false);
  c3.setNoStroke();
  c3.setStatic(true);

  c4                  = new FCircle(3);
  c4.setPosition(8, 8);
  c4.setFill(255, 255, 0);
  c4.setDensity(500);
  c4.setSensor(false);
  c4.setNoStroke();
  c4.setStatic(true);

  c5                  = new FCircle(2);
  c5.setPosition(20, 14);
  c5.setFill(255, 255, 0);
  c5.setDensity(500);
  c5.setSensor(false);
  c5.setNoStroke();
  c5.setStatic(true);

  c6                  = new FCircle(2);
  c6.setPosition(22, 10);
  c6.setFill(255, 125, 0);
  c6.setDensity(500);
  c6.setSensor(false);
  c6.setNoStroke();
  c6.setStatic(true);

  c7                  = new FCircle(3);
  c7.setPosition(28, 10);
  c7.setFill(255, 255, 0);
  c7.setDensity(500);
  c7.setSensor(false);
  c7.setNoStroke();
  c7.setStatic(true);

  c8                  = new FCircle(2);
  c8.setPosition(16, 8);
  c8.setFill(255, 255, 0);
  c8.setDensity(500);
  c8.setSensor(false);
  c8.setNoStroke();
  c8.setStatic(true);

  c9                  = new FCircle(3);
  c9.setPosition(18, 4);
  c9.setFill(255, 255, 0);
  c9.setDensity(500);
  c9.setSensor(false);
  c9.setNoStroke();
  c9.setStatic(true);


  c10                  = new FCircle(2);
  c10.setPosition(22, 5);
  c10.setFill(255, 255, 0);
  c10.setDensity(500);
  c10.setSensor(false);
  c10.setNoStroke();
  c10.setStatic(true);

  c11                  = new FCircle(3);
  c11.setPosition(15, 15);
  c11.setFill(255, 255, 0);
  c11.setDensity(500);
  c11.setSensor(false);
  c11.setNoStroke();
  c11.setStatic(true);
  
  c12                  = new FCircle(2);
  c12.setPosition(28, 15);
  c12.setFill(255, 255, 0);
  //c12.setDensity(500);
  //c12.setSensor(false);
  c12.setNoStroke();
  c12.setStatic(true);

  c13                  = new FCircle(2);
  c13.setPosition(25, 13);
  c13.setFill(255, 255, 0);
  c13.setDensity(500);
  c13.setSensor(false);
  c13.setNoStroke();
  c13.setStatic(true);

  c14                  = new FCircle(3);
  c14.setPosition(3, 4);
  c14.setFill(255, 255, 0);
  c14.setDensity(500);
  c14.setSensor(false);
  c14.setNoStroke();
  c14.setStatic(true);

  c15                  = new FCircle(2);
  c15.setPosition(2, 8);
  c15.setFill(255, 255, 0);
  c15.setDensity(500);
  c15.setSensor(false);
  c15.setNoStroke();
  c15.setStatic(true);

  c16                  = new FCircle(2);
  c16.setPosition(10, 4);
  c16.setFill(255, 125, 0);
  c16.setDensity(500);
  c16.setSensor(false);
  c16.setNoStroke();
  c16.setStatic(true);

  c17                  = new FCircle(2);
  c17.setPosition(28, 4);
  c17.setFill(255, 125, 0);
  c17.setDensity(500);
  c17.setSensor(false);
  c17.setNoStroke();
  c17.setStatic(true);

  world.add(c1);
  world.add(c2);
  world.add(c3);
  world.add(c4);
  world.add(c5);
  world.add(c6);
  world.add(c7);
  world.add(c8);
  world.add(c9);
  world.add(c10);
  world.add(c11);
  world.add(c12);
  world.add(c13);
  world.add(c14);
  world.add(c15);
  world.add(c16);
  world.add(c17);


  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(0); 
  s.h_avatar.setSensor(false);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)

  world.draw();

  /* setup framerate speed */
  frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if (renderingForce == false) {
    background(255);
    world.draw();

    textAlign(CENTER);
    fill(0, 102, 153, 204);
    textSize(18);
    text("Use number keys from 1-4 to change the setting", 220, 50);
    textAlign(LEFT);
  }
}
/* end draw section ****************************************************************************************************/

/* keyboard inputs ********************************************************************************************************/
void keyPressed() {
  /*reset*/
  if (key == '1') {
    s.h_avatar.setSensor(false);
    level1=Boolean.TRUE;
    level2=Boolean.FALSE;
    level3=Boolean.FALSE;
    level4=Boolean.FALSE;
  }
  if (key == '2') {
    s.h_avatar.setSensor(true);
    level1=Boolean.FALSE;
    level2=Boolean.TRUE;
    level3=Boolean.FALSE;
    level4=Boolean.FALSE;
  }
  if (key == '3') {
    s.h_avatar.setSensor(true);
    level1=Boolean.FALSE;
    level2=Boolean.FALSE;
    level3=Boolean.TRUE;
    level4=Boolean.FALSE;
  }
  if (key == '4') {
    s.h_avatar.setSensor(true);
    level1=Boolean.FALSE;
    level2=Boolean.FALSE;
    level3=Boolean.FALSE;
    level4=Boolean.TRUE;
  }
}


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;

    if (haplyBoard.data_available()) {
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();

      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));
    }

    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();

    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons



    /* Viscous layer codes */
    if(level1){
        s.h_avatar.setDamping(700);
    }else if (level2) {
      if (s.h_avatar.isTouchingBody(c1) || s.h_avatar.isTouchingBody(c2) || s.h_avatar.isTouchingBody(c3) || s.h_avatar.isTouchingBody(c4) || s.h_avatar.isTouchingBody(c5) || s.h_avatar.isTouchingBody(c6) || s.h_avatar.isTouchingBody(c7) || s.h_avatar.isTouchingBody(c8) || s.h_avatar.isTouchingBody(c9) || s.h_avatar.isTouchingBody(c10) || s.h_avatar.isTouchingBody(c11) || s.h_avatar.isTouchingBody(c12) || s.h_avatar.isTouchingBody(c13) || s.h_avatar.isTouchingBody(c14) || s.h_avatar.isTouchingBody(c15) || s.h_avatar.isTouchingBody(c16) || s.h_avatar.isTouchingBody(c17)) {
        PVector posDiff = (posEE.copy()).sub(posEELast);
        posEELast.set(posEE);
        if ((posDiff.mag()) < limit) { 
          s.h_avatar.setDamping(800);
          fEE.x = random(-2, 2);
          fEE.y = random(-2, 2);
        }
      }
    }else if(level3){
      s.h_avatar.setDamping(950);
    }else if (level4){
      if (s.h_avatar.isTouchingBody(c1) || s.h_avatar.isTouchingBody(c2) || s.h_avatar.isTouchingBody(c3) || s.h_avatar.isTouchingBody(c4) || s.h_avatar.isTouchingBody(c5) || s.h_avatar.isTouchingBody(c6) || s.h_avatar.isTouchingBody(c7) || s.h_avatar.isTouchingBody(c8) || s.h_avatar.isTouchingBody(c9) || s.h_avatar.isTouchingBody(c10) || s.h_avatar.isTouchingBody(c11) || s.h_avatar.isTouchingBody(c12) || s.h_avatar.isTouchingBody(c13) || s.h_avatar.isTouchingBody(c14) || s.h_avatar.isTouchingBody(c15) || s.h_avatar.isTouchingBody(c16) || s.h_avatar.isTouchingBody(c17)) {
        forceAngle = random(-1,1);
        fEE.add(3.4 * cos(forceAngle) , 3.4 * sin(forceAngle));
      }
    }

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();

    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/

/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
