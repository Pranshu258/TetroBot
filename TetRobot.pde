// 6491-2019-P1
// Base-code: Jarek ROSSIGNAC
// Student 1: Harish Krupo KPS
// Student 2: Pranshu Gupta
import processing.pdf.*;    // to save screen shots as PDFs, does not always work: accuracy problems, stops drawing or messes up some curves !!!
import java.awt.Toolkit;
import java.awt.datatransfer.*;
//  ******************* Basecode for P2 ***********************
Boolean
  rolling=true,
  animating=false,
  tracking=true,
  showTet=false,
  showTiles=true,
  showBody=true,
  smooth=true,
  interactive=true,
  PickedFocus=false,
  center=true,
  track=false,
  showLegs=true,
  showArrow=true,
  showPreviousTiles=false,
  showGhost=false;
float
  rBase=200, // radius of base triangle
  t=0,
  s=0,
  e=1;
int
  f=0, maxf=50, level=4, method=5, k=0;


enum ProgramState {
  CREATION_MODE,
  ANIMATION_MODE
};

enum CreationMode {
  CENTER_SELECTION,
  RADIUS_UPDATE
}

ProgramState PS = ProgramState.CREATION_MODE;
CreationMode CM = CreationMode.CENTER_SELECTION;

ArrayList<Obstacle> obstacles = new ArrayList<Obstacle>();
Obstacle tempObstacle;
Obstacle TARGET;

String S = "";
// S="RLRLRRRRLRLRLLLLRLRLRRRRLRLRLLLLRLRLRS";
// S="LRLRRRLRLRRRLRLRRRLRLRLRLRRRLRLRRRLRLRRRLRLR";
//S="LRLRLRRLRLRRLRLRLRLLRLRLL";
// S="LLLRRRLLLRRRORORORLLLRRRLLL";
float defectAngle=0;
//pts P = new pts(); // polyloop in 3D
//pts Q = new pts(); // second polyloop in 3D



void ShowObstacles () {
  for (int i = 0; i < obstacles.size(); i++) {
    obstacles.get(i).draw(grey);
  }
}

void ShowTarget () {
  if (TARGET != null)
    TARGET.draw(green);
}


void setup() {
  size(1000, 1000, P3D); // P3D means that we will do 3D graphics
  //size(600, 600, P3D); // P3D means that we will do 3D graphics
  myFace = loadImage("data/photo.jpg");  // load image from file pic.jpg in folder data *** replace that file with your pic of your own face
  textureMode(NORMAL);
  noSmooth();
  frameRate(30);
  GHOST.declareBot(obstacles);
  BOT.declareBot(obstacles);
  BOT.resetBot(rBase);
  GHOST.resetBot(rBase);
  _LookAtPt.reset(BOT.BotCentroid(), 10);
}

void draw() {
  background(255);
  hint(ENABLE_DEPTH_TEST);
  pushMatrix();   // to ensure that we can restore the standard view before writing on the canvas
  setView();  // see pick tab
  showFloor(); // draws dance floor as yellow mat
  doPick(); // sets Of and axes for 3D GUI (see pick Tab)

  ShowObstacles();
  ShowTarget();

  if (PS == ProgramState.ANIMATION_MODE) {
    if (!animating)
      animating = BOT.checkMove();

    if (animating) {
      if (S.charAt(0) == 'L' || S.charAt(0) == 'O' || S.charAt(0) == 'R') {
        if (f == 0)
          BOT.TriggerMotionStart(0);

        f++; // advance frame counter
        if (smooth) t=-cos(PI*f/maxf); // smooth time from -1 to 1
        else t=2.*f/maxf-1;
        if (rolling) BOT.HalfRollBot(t, 15); // rotate from mid-course position above pivot-edge
        BOT.BotCentroid();
        if (tracking&&!mousePressed)   F =_LookAtPt.move(BOT.BotCentroid()); // F = BOT.BotCentroid();
        if (f>maxf) {// if end of step
          f=0;
          BOT.TriggerMotionComplete(15);
          animating = false;
        }
      }
    }
  }

  if (showPreviousTiles) BOT.RenderPreviousTiles(5, 15);
  BOT.showBot(5, 15);

  if (showGhost)
    BOT.RenderGhostTrail(5, 15, grey);

  if (!animating && !interactive) BOT.showBot(5, 15, yellow, 255);


  popMatrix(); // done with 3D drawing. Restore front view for writing text on canvas
  hint(DISABLE_DEPTH_TEST); // no z-buffer test to ensure that help text is visible
  if (S != null)
    scribeHeader("S="+S+", path="+S.substring(0, min(k, S.length()))+", k="+k+", t="+nf(t, 1, 3), 1);

  // used for demos to show red circle when mouse/key is pressed and what key (disk may be hidden by the 3D model)
  if (mousePressed) {
    stroke(cyan);
    strokeWeight(3);
    noFill();
    ellipse(mouseX, mouseY, 20, 20);
    strokeWeight(1);
  }
  if (keyPressed) {
    stroke(red);
    fill(white);
    ellipse(mouseX+14, mouseY+20, 26, 26);
    fill(red);
    text(key, mouseX-5+14, mouseY+4+20);
    strokeWeight(1);
  }
  if (scribeText) {
    fill(black);
    displayHeader();
  } // dispalys header on canvas, including my face
  if (scribeText && !filming) displayFooter(); // shows menu at bottom, only if not filming
  if (filming && (animating || change)) saveFrame("FRAMES/F"+nf(frameCounter++, 4)+".tif");  // save next frame to make a movie
  change=false; // to avoid capturing frames when nothing happens (change is set uppn action)
  change=true;
}
