void keyPressed()
  {
    if (PS == ProgramState.CREATION_MODE && CM == CreationMode.RADIUS_UPDATE) {
      if (key=='j') {
        tempObstacle.radius = max(0, tempObstacle.radius-10);
      }
      if (key == 'k') {
        tempObstacle.radius += 10;
      }
      if (key == 'h')
        CM = CreationMode.CENTER_SELECTION;
    }

    if(key == 'D') {
      if (PS == ProgramState.CREATION_MODE)
        PS = ProgramState.ANIMATION_MODE;
      else if (PS == ProgramState.ANIMATION_MODE)
          PS = ProgramState.CREATION_MODE;
    }
  if(key=='!') snapPicture();
  if(key=='~') filming=!filming;
  if(key=='?') scribeText=!scribeText;
  if(key=='i') {interactive=!interactive;}
  if(key=='l') showLegs=!showLegs;
  if(key=='^') showTet=!showTet;
  if(key=='#') showTiles=!showTiles;
  if(key=='v') tracking=!tracking;
  if(key=='-') smooth=!smooth;
  if(key=='>') maxf/=2;
  if(key=='<') maxf*=2;
  if(key=='b') showBody=!showBody;
  if(key=='A') showArrow=!showArrow;
  // if(key=='.') k=min(k+1, S.length()-1);
  // if(key==',') k=max(0,k-1);
  if(key=='a') {animating=!animating; }// toggle animation
  if(key=='r') {BOT.resetBot(rBase); k=0; f=0; t=0; }
  if(key=='g') {rolling=true; animating=true; f=0; t=0; BOT.resetBot(rBase);} //BOT.RollBot();
  if(key=='L') {S=S+"L"; animating = true;}
  if(key=='R') {S=S+"R"; animating = true;}
  if(key=='O') {S=S+"O"; animating = true;}
  if(key=='Z') {S=S.substring(0,S.length()-1); k=S.length();}
  if(key=='C') {S=getClipboard(); k=S.length(); f=0;}
  if(key=='S') setClipboard(S);
  if(key=='_') {S=""; BOT.resetBot(rBase); k=0; f=0; t=0; }
  if(key=='G') {showGhost = !showGhost;}
  if(key=='P') {showPreviousTiles = !showPreviousTiles;}
  //if(key=='#') exit();
  if(key=='=') {}
  change=true;   // to save a frame for the movie when user pressed a key

  if (interactive && !animating) {
    switch (key) {
    case '.':
      if (k < S.length()) {
        animating = true;
      }
      break;
    case ',':
      k = max(0, k - 1);
      animating = true;
      break;
    }
  }
  }

void mouseWheel(MouseEvent event)
  {
  dz -= 50 * event.getAmount();
  change=true;
  }

void mousePressed()
  {
    if (PS == ProgramState.CREATION_MODE && CM == CreationMode.CENTER_SELECTION) {
      tempObstacle = new Obstacle(mouseX, mouseY);
      obstacles.add(tempObstacle);
      CM = CreationMode.RADIUS_UPDATE;
    }
    if (PS == ProgramState.ANIMATION_MODE) {
      TARGET = new Obstacle(mouseX, mouseY);
    }
  change=true;
  }

void mouseMoved()
  {
  //if (!keyPressed)
  if (keyPressed && key==' ') {rx-=PI*(mouseY-pmouseY)/height; ry+=PI*(mouseX-pmouseX)/width;};
  if (keyPressed && key=='`') dz+=(float)(mouseY-pmouseY); // approach view (same as wheel)
  change=true;
  }

void mouseDragged()
  {
  if (keyPressed && key=='t')  // move focus point on plane
    {
    if(center) F.sub(ToIJ(V((float)(mouseX-pmouseX),(float)(mouseY-pmouseY),0)));
    else F.add(ToIJ(V((float)(mouseX-pmouseX),(float)(mouseY-pmouseY),0)));
    }
  if (keyPressed && key=='T')  // move focus point vertically
    {
    if(center) F.sub(ToK(V((float)(mouseX-pmouseX),(float)(mouseY-pmouseY),0)));
    else F.add(ToK(V((float)(mouseX-pmouseX),(float)(mouseY-pmouseY),0)));
    }
  change=true;
  }

// **** Header, footer, help text on canvas
void displayHeader()  // Displays title and authors face on screen
    {
    scribeHeader(title,0); scribeHeaderRight(name);
    fill(white); image(myFace, width-myFace.width/2,25,myFace.width/2,myFace.height/2);
    }
void displayFooter()  // Displays help text at the bottom
    {
    scribeFooter(guide,1);
    scribeFooter(menu,0);
    }

String title ="CS6491-2019-P1: Tetrabot", name ="Harish Krupo KPS, Pranshu Gupta", // STUDENT: PUT YOUR NAMES HERE !!!
       menu="?:help, !:picture, ~:(start/stop)filming, space:rotate, `/wheel:closer, t/T:target, v:tracking, </>:slower/faster",
       guide="C/S:read/save S from/to cliboard, L/O/R:append, Z:undo, g:go, r:reset, i:interactive, ,/.:forward/backward a:animation, b:body, l:legs, ^:tets, #:tiles"; // user's guide
