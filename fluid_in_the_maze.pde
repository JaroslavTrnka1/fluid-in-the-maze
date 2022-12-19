// Maze generator
// Fluid simulation (Navier-Stokes equations) inside the maze
// By Jaroslav Trnka, 2021
// https://github.com/JaroslavTrnka1
// jaroslav_trnka@centrum.cz

// With help of:
// Daniel Shiffman
// https://thecodingtrain.com/CodingChallenges/132-fluid-simulation.html
// Real-Time Fluid Dynamics for Games by Jos Stam
// http://www.dgp.toronto.edu/people/stam/reality/Research/pdf/GDC03.pdf
// Fluid Simulation for Dummies by Mike Ash
// https://mikeash.com/pyblog/fluid-simulation-for-dummies.html

import java.util.Collections;

float dt;
float diff;
float visc;
float a;
int xside;
int yside;
int actualposition;
int start, target;
int direction;
ArrayList <Integer> tovisit = new ArrayList<Integer> ();  //záznam nehotových míst bludiště
ArrayList <Integer> track = new ArrayList<Integer> ();  //list pro záznam cesty při tvoření bludiště
ArrayList <cell> field = new ArrayList <cell> ();
ArrayList <pixel> pixelfield = new ArrayList <pixel> ();
boolean destroyed;
boolean pathfinished;
boolean contacted;
int numdestroy;
int source;

final int grid = 5;
final int N = 20;
final int scale = 5;  //velikost jednoho pixelu
int iter = 10;

void settings() {
  size(N*scale*grid, N*scale*grid);
}

void setup() {
  dt = 0.2;
  diff = 1;
  source = 4852;
  a = dt * diff;
  actualposition = 0;  //záznam pozice pro procházení bludištěm
  destroyed = false;
  contacted = false;
  numdestroy = 30;
  xside = N;   //počet čtverců na xstraně
  yside = N;
  for (int i = 1; i < (yside * xside); i++) {tovisit.add(i);}
  for (int j = 0; j < (yside * xside); j++) {field.add(new cell(j));}
  for (int q = 0; q < yside; q++) {
    for (int r = 0; r < grid; r++) {
      for (int s = 0; s < xside; s++) {
        for (int t = 0; t < grid; t++) {
          pixelfield.add(new pixel(q * xside + s, grid * s + t, grid * q + r));  
        }
      }
    }   
  }
  track.add(0);
}

void draw () {
 background(255);
 if (tovisit.size() == 0) {
   if (!destroyed) {
     destroy();
   }
   if (!contacted) {
     for (pixel p : pixelfield) {
       p.makecontacts();
     }
     contacted = true;
     for (pixel p : pixelfield) {
       p.makecorners(); 
     }
   }
   for (pixel p : pixelfield) {
     p.pixeldraw(); 
   }
   for (int i = -1; i <= 1; i++) {   //tady je zdroj density
    for (int j = -1; j <= 1; j++) {
      float added = random(50, 150);
      addDensity(source + i + (j * grid * xside), added);
    }
   }
    for (int i = 0; i < 2; i++) {  //tady je zdroj velocity
      PVector v = new PVector(mouseX - scale * pixelfield.get(source).pixelpos.x, mouseY - scale * pixelfield.get(source).pixelpos.y);
      v.normalize();
      v.mult(20);
      addVelocity(source, v.x, v.y );
    }
    pixfield_step (); 
 }
 else {
   field.get(actualposition).move();
   fill(200,0,0);
   ellipse((width/xside)*(0.5 + field.get(actualposition).pos.x), (height/yside)*(0.5 + field.get(actualposition).pos.y), 5, 5);
   for (cell c : field) {
     c.celldraw(); 
   }
 }
}


void mousePressed (){
  int pixelindex = (int(mouseX/scale) + int(mouseY/scale) * xside * grid);
  print("index: ");
  println(pixelindex);
  source = pixelindex;
}

void destroy() {
   for (int i  = 0; i < numdestroy; i++) {
     int d = int(random(field.size() - xside));
     int r = xside * int(random(yside)) + int(random(xside-1));
     if (field.get(d).downwall == true) {
       field.get(d).downwall = false;
       field.get(d).exits.add(d + xside);
       field.get(d + xside).exits.add(d);
     }
     if (field.get(r).rightwall == true) {
       field.get(r).rightwall = false;
       field.get(r).exits.add(r + 1);
       field.get(r + 1).exits.add(r);
     }     
   }
   destroyed = true;
}

class pixel {
  ArrayList <Integer> contacts;   //nenavštívení sousedé
  ArrayList <Integer> walls;
  ArrayList <Integer> corners;
  PVector pixelpos;
  int pixelid;
  int mothercell;

  float density;
  float prevdensity;

  float Vx;
  float Vy;

  float Vx0;
  float Vy0;
  
  float div;
  float project_div;
 
 
  pixel (int cellindex, int xpos, int ypos) {
    mothercell = cellindex;
    contacts = new ArrayList <Integer> (); 
    walls = new ArrayList <Integer> ();
    corners = new ArrayList <Integer> ();
    pixelpos = new PVector(xpos, ypos);
    //if (cellindex < 4) {println (pixelpos);}
    pixelid = int((pixelpos.y * xside * grid) + pixelpos.x);
    density = 0;
    prevdensity = 0;
    Vx = 0;
    Vy = 0;
    Vx0 = 0;
    Vy0 = 0;
  }
 
  void makecontacts() {
    Collections.addAll(contacts, pixelid - 1, pixelid + 1, pixelid - (xside * grid), pixelid + (xside * grid));
    for (int i = contacts.size() - 1; i >= 0; i--) {   //maže kontakty mimo plochu
      if ((contacts.get(i) < 0) || (contacts.get(i) >= (xside * yside * grid * grid))) {
        contacts.remove(i);
      }
    }
    for (int i = contacts.size() - 1; i >= 0; i--) { //maže kontakty za zdmi - nejsou ve stejné buňce nebo v buňce, která s touto otevřeně sousedí
      int mother = pixelfield.get(contacts.get(i)).mothercell;
      if (!((field.get(mothercell).exits.contains(mother)) || (mother == mothercell))) {
        contacts.remove(i);
      }
    }
    // Doplnění zdí: 1 - up, 2 - right, 3 - down, 4 - left
    if (! contacts.contains(pixelid - (xside * grid))) {walls.add(1);} 
    if (! contacts.contains(pixelid + 1)) {walls.add(2);}
    if (! contacts.contains(pixelid + (xside * grid))) {walls.add(3);}
    if (! contacts.contains(pixelid - 1)) {walls.add(4);}
  }
  
  void makecorners() {///???????
    if (!(walls.contains(1) || walls.contains(2))) {
      this.corners.add(1);
      for (int c : this.contacts) {
        if (pixelfield.get(int(((pixelpos.y-1) * xside * grid) + pixelpos.x +1)).contacts.contains(c)) {
          corners.remove(corners.indexOf(1));
          break;
        }
      }
    }
    if (!(walls.contains(2) || walls.contains(3))) {
      this.corners.add(2);
      for (int c : this.contacts) {
        if (pixelfield.get(int(((pixelpos.y+1) * xside * grid) + pixelpos.x +1)).contacts.contains(c)) {
          corners.remove(corners.indexOf(2));
          break;
        }
      }
    }
    if (!(walls.contains(3) || walls.contains(4))) {
      this.corners.add(3);
      for (int c : this.contacts) {
        if (pixelfield.get(int(((pixelpos.y+1) * xside * grid) + pixelpos.x -1)).contacts.contains(c)) {
          corners.remove(corners.indexOf(3));
          break;
        }
      }
    }
    if (!(walls.contains(4) || walls.contains(1))) {
      this.corners.add(4);
      for (int c : this.contacts) {
        if (pixelfield.get(int(((pixelpos.y-1) * xside * grid) + pixelpos.x -1)).contacts.contains(c)) {
          corners.remove(corners.indexOf(4));
          break;
        }
      }
    }
  }
  
  void dens_diffuse () {
     float sum = 0;
     for (int c : contacts) {
       sum += pixelfield.get(c).density;
     }
     this.density = (this.prevdensity + a * sum)/(1 + a * this.contacts.size());
  }
  
  void vx_diffuse () {
     float sum = 0;
     for (int c : contacts) {
       sum += pixelfield.get(c).Vx;
     }
     this.Vx = (this.Vx0 + a * sum)/(1 + a * this.contacts.size());
  }
  
  void vy_diffuse () {
     float sum = 0;
     for (int c : contacts) {
       sum += pixelfield.get(c).Vy;
     }
     this.Vy = (this.Vy0 + a * sum)/(1 + a * this.contacts.size());
  }
  
  void advect () {  
      float tmp1 = dt * Vx;
      float tmp2 = dt * Vy;
      float x    = pixelpos.x - tmp1; //kouká se zpět po vektoru, odkud (z jakého čtverce) přichází proud
      float y    = pixelpos.y - tmp2;

      x = constrain(x, 0.5, (grid * xside - 1) - 0.5); //okraje plochy
      y = constrain(y, 0.5, (grid * yside - 1) - 0.5);
      
      int i0 = floor(x);    //i0,j0; i0,j1; i1,j0; i1,j1 jsou ty čtyři čtverce, odkud přichází vektor proudu
      int i1 = i0 + 1;
      int j0 = floor(y);
      int j1 = j0 + 1; 

      float s1 = x - i0;   //tady se počítají poměry zastoupení těch čtyř buněk - kde přesně mezi nimi je pata vektoru
      float s0 = 1.0 - s1; 
      float t1 = y - j0; 
      float t0 = 1.0 - t1;

      // lineární interpolace těch čtyř čtverců - kolik dají dohromady tomu cílovéu čtverci NEOŠETŘENÉ HRANICE - bylo by potřeba předělat
      this.density = s0 * (t0 * pixelfield.get(i0 + xside * grid * j0).prevdensity + t1 * pixelfield.get(i0 + xside * grid * j1).prevdensity) +
        s1 * (t0 * pixelfield.get(i1 + xside * grid * j0).prevdensity + t1 * pixelfield.get(i1 + xside * grid * j1).prevdensity); 
      this.Vx = s0 * (t0 * pixelfield.get(i0 + xside * grid * j0).Vx0 + t1 * pixelfield.get(i0 + xside * grid * j1).Vx0) +
        s1 * (t0 * pixelfield.get(i1 + xside * grid * j0).Vx0 + t1 * pixelfield.get(i1 + xside * grid * j1).Vx0); 
      this.Vy = s0 * (t0 * pixelfield.get(i0 + xside * grid * j0).Vy0 + t1 * pixelfield.get(i0 + xside * grid * j1).Vy0) +
        s1 * (t0 * pixelfield.get(i1 + xside * grid * j0).Vy0 + t1 * pixelfield.get(i1 + xside * grid * j1).Vy0); 
  }
  
  void project_set_div () {
    div = 0;
    project_div = 0;
    int h = xside * grid;
    for (int c : contacts) {   //Hrubá divergence každého pixelu
      if (pixelfield.get(c).pixelpos.x == (this.pixelpos.x + 1)) {div += h * pixelfield.get(c).Vx;}
      if (pixelfield.get(c).pixelpos.x == (this.pixelpos.x - 1)) {div -= h * pixelfield.get(c).Vx;}
      if (pixelfield.get(c).pixelpos.y == (this.pixelpos.y + 1)) {div += h * pixelfield.get(c).Vy;}
      if (pixelfield.get(c).pixelpos.y == (this.pixelpos.x - 1)) {div -= h * pixelfield.get(c).Vy;}
    }
    div = -0.5 * div;
  }
  
  void project_div () {   //tohle se bude iterovat
    int coefficient = 20*this.contacts.size();
    project_div = div/coefficient;
    for (int c : contacts) {
      this.project_div += pixelfield.get(c).project_div/coefficient;
    }
  }

  void project_fin () {
    int h = xside * grid;
    for (int c : contacts) {
      if (pixelfield.get(c).pixelpos.x == (this.pixelpos.x + 1)) {Vx -= 0.5 * pixelfield.get(c).project_div / h;}
      if (pixelfield.get(c).pixelpos.x == (this.pixelpos.x - 1)) {Vx += 0.5 * pixelfield.get(c).project_div / h;}
      if (pixelfield.get(c).pixelpos.y == (this.pixelpos.y + 1)) {Vy -= 0.5 * pixelfield.get(c).project_div / h;}
      if (pixelfield.get(c).pixelpos.y == (this.pixelpos.x - 1)) {Vy += 0.5 * pixelfield.get(c).project_div / h;}
    }
  }
  
  void vel_limit () {
    if (walls.contains(1)) {Vy = max (0, Vy);}
    if (walls.contains(2)) {Vx = min (0, Vx);}
    if (walls.contains(3)) {Vy = min (0, Vy);}
    if (walls.contains(4)) {Vx = max (0, Vx);}
    int m = Integer.signum(int(Vy)) + 2 * Integer.signum(int(Vx)); 
    if ((corners.contains(1)) && (m == 1)) {turn();}
    if ((corners.contains(2)) && (m == 3)) {turn();}
    if ((corners.contains(3)) && (m == -1)) {turn();}
    if ((corners.contains(4)) && (m == -3)) {turn();}
  }
  
  void turn () {
    if (abs(Vx) > abs(Vy)) {
      Vx += Integer.signum(int(Vx)) * abs(Vy);
      Vy = 0;
    }
    if (abs(Vx) < abs(Vy)) {
      Vy += Integer.signum(int(Vy)) * abs(Vx);
      Vx = 0;
    }
  }
   
  void pixeldraw() {
   noStroke();
   fill(255 - density/2);
   square(pixelpos.x * scale,pixelpos.y * scale, scale); 
   stroke(0,150,50);
   if (walls.contains(1)) {line(pixelpos.x * scale, pixelpos.y * scale, (pixelpos.x + 1) * scale, pixelpos.y * scale);}
   if (walls.contains(2)) {line((pixelpos.x + 1) * scale, pixelpos.y * scale, (pixelpos.x + 1) * scale, (pixelpos.y + 1) * scale);}
   if (walls.contains(3)) {line((pixelpos.x + 1) * scale, (pixelpos.y + 1) * scale, pixelpos.x * scale, (pixelpos.y + 1) * scale);}
   if (walls.contains(4)) {line(pixelpos.x * scale, (pixelpos.y + 1) * scale, pixelpos.x * scale, pixelpos.y * scale);}         
   }
}

void pixfield_project () {
  for (pixel p : pixelfield) {
    p.project_set_div();
  }
  
  for (int i = 0; i < iter; i++) {
    for (pixel p : pixelfield) {
      p.project_div();
    }
  }
  
  for (pixel p : pixelfield) {
    p.project_fin ();
  }
}

void pixfield_dens_diffuse () {
  for (int it = 0; it < iter; it++) {
    for (pixel p : pixelfield) {
      p.dens_diffuse();
    }
  }
}

void pixfield_vel_diffuse () {
  for (int it = 0; it < iter; it++) {
    for (pixel p : pixelfield) {
      p.vx_diffuse();
      p.vy_diffuse();
    }
  }
}

void pixfield_advect () {
  for (pixel p : pixelfield) {
      p.advect();
  }
}

void pixfield_step () {
  pixfield_dens_diffuse ();
  pixfield_vel_diffuse();
  pixfield_consum();
  pixfield_advect();
  pixfield_consum();
  pixfield_project();
  pixfield_consum();
}

void pixfield_consum () {
  for (pixel p : pixelfield) {
    p.vel_limit();
    p.prevdensity = p.density;  
    p.Vx0 = p.Vx;
    p.Vy0 = p.Vy;
  }
}

void addDensity(int index, float amount) {
  pixelfield.get(index).prevdensity += amount;
}

void addVelocity(int index, float amountX, float amountY) {
  pixelfield.get(index).Vx0 += amountX;
  pixelfield.get(index).Vy0 += amountY;
}

class cell{
 ArrayList <Integer> neighbours;   //nenavštívení sousedé
 ArrayList <Integer> exits;   //východy k sousedům, či
 boolean rightwall;
 boolean downwall;
 PVector pos;
 int id;
 
 cell(int index) {
   exits = new ArrayList<Integer> ();
   neighbours = new ArrayList<Integer> ();
   rightwall = true;
   downwall = true;
    if (index >= xside) {neighbours.add(index - xside);}   //index horního souseda
    if (index < (yside-1) * xside) {neighbours.add(index + xside);}  //index dlního souseda
    if ((index % xside) > 0) {neighbours.add(index - 1);}  //index levého
    if ((index % xside) < yside-1) {neighbours.add(index + 1);}  //index pravého
   pos = new PVector(index % xside, int (index / xside));   //vektor, kódující pozici v x-y síti
   id = index;
 }
 
  void move() {
    neighbours.retainAll(tovisit);   //průnik množiny sousedů a nenavštívených buněk
    if (neighbours.isEmpty()) {
      deadend();
    }
    else {
      int step = neighbours.get(int(random(neighbours.size())));
      track.add(step);
      tovisit.remove(tovisit.indexOf(step));
      exits.add(step);
      field.get(step).exits.add(actualposition);  //aby své východy vnímaly vzájemně.
      if (step - actualposition == 1) {rightwall = false;}
      if (step - actualposition == -1) {field.get(step).rightwall = false;}
      if (step - actualposition == yside) {downwall = false;}
      if (step - actualposition == -yside) {field.get(step).downwall = false;}
      actualposition = step;
    }
 }

 void celldraw() {
   stroke(0,150,50);
   if (rightwall) {line((pos.x+1) * (width/xside), pos.y * (height/yside), (pos.x+1) * (width/xside), (pos.y + 1) * (height/yside));}
   if (downwall) {line((pos.x) * (width/xside), (pos.y+1) * (height/yside), (pos.x+1) * (width/xside), (pos.y + 1) * (height/yside));}
 }
 
 void deadend() {
  actualposition = track.get(track.size()-2); 
  track.remove(track.size()-1); 
 }
}
