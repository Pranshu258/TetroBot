import java.util.Arrays;

vec Up = V(0, 0, 1);
TETROBOT BOT = new TETROBOT();
TETROBOT GHOST = new TETROBOT();

class Obstacle {
  pt center = P();
  int mx = 0, my = 0;
  float radius = 10;
  boolean picked = false;
  public Obstacle(int _mx, int _my) {
    this.mx = _mx;
    this.my = _my;
  }
  public String toString() {
    return String.format("x: %f, y: %f, z: %f, r:%f", center.x, center.y, center.z, radius);
  }
  public void draw (color c) {
    if (!picked) {
      center = pick(mx, my);
      center.z = 0;
      picked = true;
    }
    pt top = P(center);
    top.z = 1000;
    fill(c);
    cylinderSection(center, top, radius);
    caplet(center, radius, top, radius);
  }

  public boolean equals(Obstacle o) {
    return (this.radius == o.radius &&
      this.center.equals(o.center));
  }
}

class TETROBOT implements Cloneable// class for manipulaitng TETROBOTs
{
  pt[] G = new pt[4];     // hip positions
  color[] col = new color[4];
  int a=0, b=1, c=2, d=3; // I use these to remember which hip is which
  pt[] coreVerts = new pt[4];     // hip positions
  pt[] kneeVerts = new pt[4];     // knee positions
  pt[] feetVerts = new pt[4];
  float totalangle;
  float lengthOfLeg;
  pt vOpp, vTop;
  pt refMid;
  vec orthoVector;
  vec feetOrth, ofv1, ofv2, tfv1, tfv2;
  float radius;
  pt topFeetCircleCenter, oppFeetCircleCenter;
  int[] LRO = new int[3];
  Integer[] layout = new Integer[4];
  Integer[] nextLayout = new Integer[4];
  float refMidTopMagnitude, refMidOppMagnitude;
  ArrayList<pt> computedEndVertices = new ArrayList<pt>();
  ArrayList<Integer> computedEndLayouts = new ArrayList<Integer>(); // To know the colors of the vertices
  HashMap<pt, Boolean> obstacleMap = new HashMap<pt, Boolean>();
  ArrayList<Obstacle> privObstacles;
  ArrayList<pt> visited = new ArrayList<pt>();
  Obstacle prevTARGET = null;

  int pivotVert1 = 1, pivotVert2 = 0, oppVert = 2, topVert = 3;
  TETROBOT() {
  }

  public Object clone() throws CloneNotSupportedException {
    // Assign the shallow copy to new reference variable t
    TETROBOT t = (TETROBOT) super.clone();

    t.G = copyPointArray(G);
    t.layout = Arrays.copyOf(layout, 4);
    t.kneeVerts = Arrays.copyOf(kneeVerts, 4);
    t.feetVerts = Arrays.copyOf(feetVerts, 4);
    return t;
  }

  void declareBot(ArrayList<Obstacle> obs)
  {
    for (int i=0; i<4; i++) G[i]=P();
    col[0]=red; // STUDENTS: Use your personalized colors
    col[1]=blue;
    col[2]=green;
    col[3]=gold;
    this.privObstacles = new ArrayList<Obstacle>(obs);
  }

  boolean checkIfRightOf(pt P, pt A, pt B) {
    vec AP = V(A, P);
    vec AB = V(A, B);
    vec c = cross(AB, AP);
    return c.z > 0;
  }

  boolean checkInsideTile(pt A, pt B, pt C, Obstacle obstacle) {
    pt P = obstacle.center;

    return checkIfRightOf(P, A, B) &&
      checkIfRightOf(P, B, C) &&
      checkIfRightOf(P, C, A);
  }

  boolean checkIfCircleContainsPoint(Obstacle obstacle, pt A) {
    return d(A, obstacle.center) < obstacle.radius;
  }

  boolean checkIfCircleIntersectsEdge(Obstacle obstacle, pt A, pt B) {
    pt C = obstacle.center;
    vec AB = U(A, B);
    vec AC = V(A, C);
    float length = dot(AB, U(AC)) * n(AC);
    if (length < 0 || length > d(A, B))
      return false;

    pt L = P(A, length, AB);
    float lengthOfPerpendicular = n(V(C, L));
    return lengthOfPerpendicular < obstacle.radius;
  }

  void printObstacleMap() {
    for (HashMap.Entry<pt, Boolean> entry : obstacleMap.entrySet()) {
      System.out.println(entry.getKey() + " : " + entry.getValue());
    }
  }

  boolean checkIfTileIsAnObsctacle(pt A, pt B, pt C) {
    pt centroid = P(A, B, C);
    boolean test = false;
    //printObstacleMap();

    //if (obstacleMap.containsKey(centroid)) {
    //  println("Got the centroid in map\n");
    //  return obstacleMap.get(centroid);
    //}
    for (Obstacle obs : obstacles) {
      test = checkInsideTile(A, B, C, obs);

      if (!test)
        test = test || checkIfCircleContainsPoint(obs, A);
      if (!test)
        test = test || checkIfCircleContainsPoint(obs, B);
      if (!test)
        test = test || checkIfCircleContainsPoint(obs, C);

      if (!test)
        test = test || checkIfCircleIntersectsEdge(obs, A, B);
      if (!test)
        test = test || checkIfCircleIntersectsEdge(obs, B, C);
      if (!test)
        test = test || checkIfCircleIntersectsEdge(obs, C, A);

      if (test)
        break;
    }

    //obstacleMap.put(centroid, test);
    return test;
  }

  pt[] copyPointArray(pt[] in) {

    pt[] ret = new pt[in.length];
    for (int i = 0; i < in.length; i++)
      ret[i] = in[i];

    return ret;
  }

  boolean checkIfVisited(pt P) {
    for (pt itr : visited) {
      if (itr.equals(P))
        return true;
    }

    return false;
  }

  // return true if we need to animate
  boolean checkMove() {
    float minVal = Float.MAX_VALUE;
    int direction = -1;
    int dItr;
    boolean obstacleTile = false;
    int blockades = 0;

    pt A = G[layout[0]];
    pt B = G[layout[1]];
    pt C = G[layout[2]];

    if (TARGET == null)
      return false;

    if (prevTARGET != TARGET) {
      prevTARGET = TARGET;
      visited.clear();
    }

    if (checkInsideTile(A, B, C, TARGET)) {
      TARGET = null;
      return false;
    }

    visited.add(P(A, B, C));

    S = "LRO";
    for (dItr = 0; dItr < 3; dItr++) {
      try {
        GHOST = (TETROBOT) clone();
      }
      catch (CloneNotSupportedException e) {
        e.printStackTrace();
      }

      GHOST.TriggerMotionStart(dItr);
      GHOST.HalfRollBot(1.0, 15);
      GHOST.TriggerMotionComplete(15);
      A = GHOST.G[GHOST.layout[0]];
      B = GHOST.G[GHOST.layout[1]];
      C = GHOST.G[GHOST.layout[2]];
      pt centroid = P(A, B, C);

      pt goalCenter = TARGET.center;
      float distance = d(centroid, goalCenter);
      obstacleTile = checkIfTileIsAnObsctacle(A, B, C);
      boolean alreadyVisited = checkIfVisited(centroid);
      if (obstacleTile)
        blockades++;

      if (!obstacleTile &&
        (distance < minVal) &&
        !alreadyVisited) {
        minVal = distance;
        direction = dItr;
      }
    }

    if (direction < 0) {
      if (blockades == 3) {
        return false;
      } else {
        S = "O";
        return true;
      }
    }

    S = Character.toString(S.charAt(direction));
    return true;
  }

  void resetBot(float r) // puts the bot in the center of the terrain
  {
    pt X = P(); // center of circle
    for (int i=0; i<3; i++) G[i]=R(P(X, V(-r, 0, 0)), 2.*PI*i/3, X); // points on z=0 plane
    pt A=G[0], B=G[1], C=G[2];
    pt O = P(A, B, C); // centroid of triangle
    float e=(d(A, B)+d(B, C)+d(C, A))/3; // average edge length (in case we rin on non equilateral triangles)
    pt D = P(O, e*sqrt(2./3), Up);
    G[3]=D;
    a=0;
    b=1;
    c=2;
    d=3;

    layout[0] = a;
    layout[1] = b;
    layout[2] = c;
    layout[3] = d;

    computedEndVertices.clear();
    computedEndLayouts.clear();
    computedEndVertices.addAll(Arrays.asList(G));
    computedEndLayouts.addAll(Arrays.asList(layout));


    //lengthOfLeg = 0.500001*(2*d(G[0], G[1])/sqrt(3) -  d(P(G[layout[0]], G[layout[1]], G[layout[2]]), P(BotCentroid(), V(n(V(G[layout[0]], G[layout[1]]))/(4 * sqrt(3)), U(BotCentroid(), G[3])))));
    feetOrth = U(cross(V(P(G[layout[0]], G[layout[1]]), G[layout[0]]), Up));

    computeCoreVertices(r);
    lengthOfLeg = 1.100011*d(G[0], coreVerts[0]);
    float r1 = d(BotCentroid(), P(BotCentroid().x, BotCentroid().y, 0)) + d(P(coreVerts[3], 2*lengthOfLeg, Up), BotCentroid());
    radius = sqrt(r1*r1 + 4*d(G[0], G[1])*d(G[0], G[1])/3);

    pt centroidProjection = BotCentroid();
    centroidProjection.z = 0;
    pt p2 = P(centroidProjection, 2*d(G[0], G[1])/sqrt(3), V(-1.0, feetOrth)), p1 = P(coreVerts[3], 2*lengthOfLeg, Up);
    topFeetCircleCenter = P(P(p1, p2), radius*sqrt(3)/2, V(-1.0, U(B(V(p1, p2), Up))));
    tfv1 = U(topFeetCircleCenter, p2);
    tfv2 = U(P(p2, topFeetCircleCenter), p1); // goes from PI/3 to 0
    p1 = P(1, G[0], 1, G[1], -1, centroidProjection);
    p1.z = P(coreVerts[3], 2*lengthOfLeg, Up).z;
    p2 = G[2];
    oppFeetCircleCenter = P(P(p1, p2), radius*sqrt(3)/2, V(-1.0, U(B(V(p1, p2), Up))));
    ofv1 = U(oppFeetCircleCenter, p2);
    ofv2 = U(P(p2, oppFeetCircleCenter), p1); // goes from 0 to PI/3
    ComputeKneesAndFeet(-1.0);
  }

  //void placeBot(pt A, pt B, pt C, float h, float s, int a, int b, int c, int d)

  void placeBot(pt A, pt B, pt C) // puts the bot tet so that one of its facs is the triangle ABC
  {
    pt O = P(A, B, C); // centroid of triangle
    float e=(d(A, B)+d(B, C)+d(C, A))/3; // average edge length
    pt D = P(O, e*sqrt(2./3), Up);
    G[0].setTo(A);
    G[1].setTo(B);
    G[2].setTo(C);
    G[3].setTo(D);
  }

  pt BotCentroid() {
    return P(G[0], G[1], G[2], G[3]);
  } // average of the 4 vertices


  void showBot (float w, float r)
  {
    noStroke();
    if (showTet)
    {
      for (int i=0; i<4; i++) {
        fill(col[i]);
        show(G[i], r);
      } // shows tet vertices as balls
      fill(green);
      for (int v=0; v<3; v++)  for (int u=v+1; u<4; u++) caplet(G[v], w, G[u], w); // shows tet edges as cylinders
      pt X = P(G[b], 0.2, V(G[b], P(G[a], G[c])));
      pt Y = P(G[c], 0.2, V(G[c], P(G[b], G[a])));
      fill(red);
      if (showArrow) arrow(X, Y, 5); // shows arrow in the wedge of the rotation axis
    }
    if (showBody) showCore(r); // shows the tetRobot core (body)
    if (showLegs) showFeet(15);
    fill(brown);
    show(oppFeetCircleCenter, 20);
    fill(pink);
    show(topFeetCircleCenter, 20);
  }

  void showTile (float w, float r) // shows the base triangle
  {
    fill(col[a]);
    show(G[a], r);
    fill(col[b]);
    show(G[b], r);
    fill(col[c]);
    show(G[c], r);
    fill(yellow);
    caplet(G[b], w, G[c], w);
    fill(cyan);
    caplet(G[c], w, G[a], w);
    caplet(G[a], w, G[b], w);
  }

  void showBot (float w, float r, color colBeams, int opacity)
  {
    if (showTiles)
    {

      // STUDENTS
    }
    if (opacity<1) return;
    noStroke();
    if (showTet)
    {

      // STUDENTS
    }
    if (showBody) showCore(opacity);
    if (showLegs)
    {

      // STUDENTS
    }
  }

  void computeCoreVertices (float r) {
    pt centroid = BotCentroid();
    float lengthOfEdge = n(V(G[layout[0]], G[layout[1]]));
    float distFromCentroid = lengthOfEdge / (4 * sqrt(3));
    for (int i=0; i<4; i++) {
      coreVerts[i] = P(centroid, V(distFromCentroid, U(centroid, G[i])));
    }
  }

  void showCore(float r) {
    computeCoreVertices(r);
    for (int i = 0; i < 4; i++) {
      fill(col[i]);
      show(coreVerts[i], r);
      int v = i;
      fill(col[i]);
      beginShape(TRIANGLES);
      vertex(coreVerts[v].x, coreVerts[v].y, coreVerts[v].z);
      v = (v + 1) % 4;
      vertex(coreVerts[v].x, coreVerts[v].y, coreVerts[v].z);
      v = (v + 1) % 4;
      vertex(coreVerts[v].x, coreVerts[v].y, coreVerts[v].z);
      endShape();
    }
  }

  void showFeet(float w) {
    for (int i = 0; i < 4; i++) {
      fill(grey);
      leg(coreVerts[i], kneeVerts[i], 20, 15);
      leg(kneeVerts[i], feetVerts[i], 15, 10);
    }
  }

  void showCore(int opacity)
  {

    // STUDENTS
  }

  void pivotBot()
  {

    // STUDENTS
  }

  void HalfRollBot()
  {

    // STUDENTS
  }

  float AngleFromTime(float t, float tmin, float tmax, float amin, float amax) {
    float angle = 0;
    float tscale = tmax - tmin;
    float ascale = amax - amin;
    angle = (t - tmin) * ascale / tscale + amin;
    return angle;
  }

  void CalculateNextLayout(char direction) {
    nextLayout = new Integer[4];
    switch (direction) {
    case 'L':
      nextLayout[0] = layout[0];
      nextLayout[1] = layout[3];
      nextLayout[2] = layout[1];
      nextLayout[3] = layout[2];
      break;
    case 'R':
      nextLayout[0] = layout[1];
      nextLayout[1] = layout[3];
      nextLayout[2] = layout[2];
      nextLayout[3] = layout[0];
      break;
    case 'O':
      nextLayout[0] = layout[2];
      nextLayout[1] = layout[3];
      nextLayout[2] = layout[0];
      nextLayout[3] = layout[1];
      break;
    }
  }

  void TriggerMotionStart(int pos) {
    char nextPos = S.charAt(pos);
    //char nextNextPos = '_';
    //if (pos + 1 < S.length()) nextNextPos = S.charAt(pos + 1);
    if (nextPos == 'L' || nextPos == 'O' || nextPos == 'R') {
      CalculateNextLayout(nextPos);
      pt V1 = G[nextLayout[0]];
      pt V2 = G[nextLayout[2]];

      vec V1V2 = U(V1, V2);
      vec Z = V(0, 0, 1);
      orthoVector = cross(V1V2, Z);
      refMid = P(V1, V2);

      vOpp = G[nextLayout[3]];
      vTop = G[layout[3]];

      vec refMidTop = V(refMid, vTop);
      vec refMidOpp = V(refMid, vOpp);

      refMidTopMagnitude = n(refMidTop);
      refMidOppMagnitude = n(refMidOpp);

      float topangle = angle(refMidTop, Z);
      totalangle = topangle + PI / 2;

      topVert = layout[3];
      pt centroidProjection = P(BotCentroid());
      centroidProjection.z = 0;
      if (nextPos == 'R') {
        pivotVert1 = layout[2];
        pivotVert2 = layout[1];
        oppVert = layout[0];
        feetOrth = U(cross(V(P(G[layout[1]], G[layout[2]]), G[layout[1]]), Up));
      } else if (nextPos == 'L') {
        pivotVert1 = layout[1];
        pivotVert2 = layout[0];
        oppVert = layout[2];
        feetOrth = U(cross(V(P(G[layout[0]], G[layout[1]]), G[layout[0]]), Up));
        //oppFeetCircleCenter = P(1, G[pivotVert1], 1, G[pivotVert2], -1, centroidProjection);
      } else if (nextPos == 'O') {
        pivotVert1 = layout[0];
        pivotVert2 = layout[2];
        oppVert = layout[1];
        feetOrth = U(cross(V(P(G[layout[0]], G[layout[2]]), G[layout[2]]), Up));
        //oppFeetCircleCenter = P(1, G[pivotVert1], 1, G[pivotVert2], -1, centroidProjection);
      }
      pt p2 = P(centroidProjection, 2*d(G[0], G[1])/sqrt(3), V(-1.0, feetOrth)), p1 = P(coreVerts[topVert], 2*lengthOfLeg, Up);
      topFeetCircleCenter = P(P(p1, p2), radius*sqrt(3)/2, V(-1.0, U(B(V(p1, p2), Up))));

      tfv1 = U(topFeetCircleCenter, p2);
      tfv2 = U(P(p2, topFeetCircleCenter), p1); // goes from PI/3 to 0
      p1 = P(1, G[pivotVert1], 1, G[pivotVert2], -1, centroidProjection);
      p1.z = P(coreVerts[topVert], 2*lengthOfLeg, Up).z;
      p2 = G[oppVert];
      oppFeetCircleCenter = P(P(p1, p2), radius*sqrt(3)/2, V(-1.0, U(B(V(p1, p2), Up))));
      ofv1 = U(oppFeetCircleCenter, p2);
      ofv2 = U(P(p2, oppFeetCircleCenter), p1); // goes from 0 to PI/3
    }
  }

  void RenderPreviousTiles(float w, float r) {
    int i, v1, v2, v3;
    pt P1, P2, P3;
    for (i = 0; i < computedEndVertices.size(); i += 4) {
      v1 = computedEndLayouts.get(i);
      v2 = computedEndLayouts.get(i + 1);
      v3 = computedEndLayouts.get(i + 2);
      P1 = computedEndVertices.get(i + v1);
      P2 = computedEndVertices.get(i + v2);
      P3 = computedEndVertices.get(i + v3);

      fill(col[v1]);
      show(P1, r);
      fill(col[v2]);
      show(P2, r);
      fill(col[v3]);
      show(P3, r);
      fill(cyan);
      caplet(P2, w, P3, w);
      fill(cyan);
      caplet(P3, w, P1, w);
      caplet(P1, w, P2, w);
    }
  }

  void RenderGhostTrail(float w, float r, color colBeams) {
    int i, v1, v2, v3, v4;
    pt P1, P2, P3, P4;
    int trailLength = computedEndVertices.size() / 4;
    // The alpha value goes from 50 to 255 in divisions of the trailLength
    float opacity = 50;
    float alphaStep = (255 - 50) / trailLength;
    for (i = 0; i < computedEndVertices.size(); i += 4) {
      v1 = computedEndLayouts.get(i);
      v2 = computedEndLayouts.get(i + 1);
      v3 = computedEndLayouts.get(i + 2);
      v4 = computedEndLayouts.get(i + 3);
      P1 = computedEndVertices.get(i + v1);
      P2 = computedEndVertices.get(i + v2);
      P3 = computedEndVertices.get(i + v3);
      P4 = computedEndVertices.get(i + v4);

      fill(col[v1], opacity);
      show(P1, r);
      fill(col[v2], opacity);
      show(P2, r);
      fill(col[v3], opacity);
      show(P3, r);
      fill(col[v4], opacity);
      show(P4, r);

      fill(colBeams, opacity);
      caplet(P2, w, P3, w);
      caplet(P3, w, P1, w);
      caplet(P1, w, P2, w);
      caplet(P1, w, P4, w);
      caplet(P2, w, P4, w);
      caplet(P3, w, P4, w);
      opacity += alphaStep;
    }
  }

  void HalfRollBot(float t, float r)
  {
    float vOppAngle, vTopAngle;
    vec genOpp, genTop;
    vec Z = V(0, 0, 1);

    vOppAngle = AngleFromTime(t, -1, 1, PI, PI - totalangle);
    vTopAngle = AngleFromTime(t, -1, 1, totalangle, 0);

    genOpp = S(V(refMidOppMagnitude * cos(vOppAngle), orthoVector), 1, V(refMidOppMagnitude * sin(vOppAngle), Z));
    genTop = S(V(refMidTopMagnitude * cos(vTopAngle), orthoVector), 1, V(refMidTopMagnitude * sin(vTopAngle), Z));

    G[nextLayout[3]] = P(refMid, genOpp);
    G[layout[3]] = P(refMid, genTop);

    ComputeKneesAndFeet(t);
  }

  void ComputeKneesAndFeet (float t) {
    // pivotVerts
    feetVerts[pivotVert1] = G[pivotVert1];
    feetVerts[pivotVert2] = G[pivotVert2];

    // for the flying feet
    float fOppAngle = t <= 0 ? 0 : t*PI/3;
    float fTopAngle = t > 0 ? 0 : -t*PI/3;
    feetVerts[oppVert] = P(oppFeetCircleCenter, S(V(radius*cos(fOppAngle), ofv1), 1, V(radius*sin(fOppAngle), ofv2)));
    feetVerts[topVert] = P(topFeetCircleCenter, S(V(radius*cos(fTopAngle), tfv1), 1, V(radius*sin(fTopAngle), tfv2)));
    for (int i = 0; i < 4; i++) {
      float coreFeetDist = d(coreVerts[layout[i]], feetVerts[layout[i]]);
      kneeVerts[layout[i]] = P(P(coreVerts[layout[i]], feetVerts[layout[i]]), sqrt(lengthOfLeg*lengthOfLeg - coreFeetDist*coreFeetDist/4), U(B(V(coreVerts[layout[i]], feetVerts[layout[i]]), Up)));
    }
  }

  void TriggerMotionComplete(float r) {
    HalfRollBot(1.0, r);
    layout = nextLayout;
    computedEndVertices.addAll(Arrays.asList(G));
    computedEndLayouts.addAll(Arrays.asList(layout));
  }

  void RollBot()
  {

    // STUDENTS
  }

  void placeBot(pt A, pt B, pt C, int pa, int pb, int pc, int pd)
  {

    // STUDENTS
  }



  void placeBot(pt A, pt B, int pa, int pb, int pc, int pd)
  {

    // STUDENTS
  }

  void placeBot(pt B, pt C)
  {
  }
} // END TETROBOT CLASS
