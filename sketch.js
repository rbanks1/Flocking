// A likely bad translation of Daniel Shiffman's Flocking code
// from Processing.js to P5.js
// See original: https://processing.org/examples/flocking.html
// Using P5.Gui. Copyright (c) 2016 Martin Schneider @bitcraftlab https://github.com/bitcraftlab/p5.gui
// By Richard Banks. January 2021.

//#region GLOBAL VARIABLES
var flock;
var pillars = []; // An array for the static pillars
var createBoidOnClick = true;
var numberOfBoidsToCreateAtStart = -1;
var lastPillarPosition = [0, 0];

var maxspeed = 3; // Maximum speed
var maxspeedMin = 0;
var maxspeedMax = 50;
var maxspeedStep = 1;

var maxforce = 0.03; // Maximum steering force
var maxforceMin = 0.01;
var maxforceMax = 0.4;
var maxforceStep = 0.01;

var rScale = 2;
var rScaleMin = 0.1;
var rScaleMax = 20;
var rScaleStep = 0.1;

var seperation = 1.5;
var seperationMin = 0;
var seperationMax = 20;
var seperationStep = 0.1;

var seperationPillars = 10;
var seperationPillarsMin = 0;
var seperationPillarsMax = 20;
var seperationPillarsStep = 0.1;

var alignment = 1.0;
var alignmentMin = 0;
var alignmentMax = 2;
var alignmentStep = 0.1;

var cohesion = 1.0;
var cohesionMin = 0;
var cohesionMax = 2;
var cohesionStep = 0.1;

var gui; // P5.GUI control
var guiVisible;

//#endregion

// *** SETUP ***
function setup() {
  createCanvas(windowWidth, windowHeight);

  gui = createGui("Settings").setPosition(10, 140);
  gui.addGlobals(
    "maxspeed",
    //"maxforce",
    //"rScale",
    "seperation",
    "seperationPillars"
    //"alignment",
    //"cohesion",
    //"createBoidOnClick"
  );
  gui.hide();
  guiVisible = false;

  flock = new Flock();
  // Add an initial set of boids into the system
  for (var i = 0; i < numberOfBoidsToCreateAtStart; i++) {
    flock.addBoid(new Boid(width / 2, height / 2));
  }
}

// *** DRAW ***
function draw() {
  background(50);

  fill(200);
  noStroke();
  text("S: Save Canvas", 10, 20);
  text("G: Show/Hide Settings", 10, 80);
  text("DEL: Clear everything", 10, 100);
  if(createBoidOnClick) fill(255);
  else fill(200);
  text("B: Click to create bird [" + flock.boids.length + "]", 10, 40);
  if(!createBoidOnClick) fill(255);
  else fill(200);
  text("P: Click to create pillar [" + pillars.length + "]", 10, 60);

  flock.run();
  if (mouseIsPressed && mouseButton == LEFT && !createBoidOnClick) {
    var d = dist(lastPillarPosition[0], lastPillarPosition[1], mouseX, mouseY);
    if (d > 10) {
      pillars.push(new Pillar(mouseX, mouseY));
      lastPillarPosition = [mouseX, mouseY];
    }
  }
  pillars.forEach((pillar) => {
    pillar.render();
  });
}

// *** EVENTS ***
// -------------------------------

// EVENT mousePressed 
function mousePressed(event) {
  if (createBoidOnClick) flock.addBoid(new Boid(mouseX, mouseY));
}

// EVENT windowResized
function windowResized() {
  resizeCanvas(windowWidth, windowHeight);
}

// EVENT keyReleased 
function keyReleased() {
  if (key == "s" || key == "S") saveCanvas("flocking", "png");
  if (key == "b" || key == "B") createBoidOnClick = true;
  if (key == "p" || key == "P") createBoidOnClick = false;
  if (key == "g" || key == "G") {
    if (guiVisible) {
      gui.hide();
      guiVisible = false;
    } else {
      gui.show();
      guiVisible = true;
    }
  }
  if (keyCode == DELETE || keyCode == BACKSPACE) {
    flock.clear();
    pillars = [];
  }
}

// *** CLASSES ***
// --------------------------------

// CLASS Flock (a list of Boid objects)
class Flock {
  constructor() {
    this.boids = []; // Initialize the ArrayList
  }

  run() {
    this.boids.forEach((b) => {
      b.run(this.boids); // Passing the entire list of boids to each boid individually
    });
  }

  addBoid(b) {
    this.boids.push(b);
  }

  clear() {
    this.boids = [];
  }
}

// CLASS Boid (a flocking entity)
class Boid {
  constructor(x, y) {
    this.acceleration = createVector(0, 0);
    this.position = createVector(x, y);
    this.angle = random(TWO_PI);
    this.velocity = createVector(cos(this.angle), sin(this.angle));
  }

  run(boids) {
    this.flock(boids);
    this.update();
    this.borders();
    this.render();
  }

  applyForce(force) {
    // We could add mass here if we want A = F / M
    this.acceleration.add(force);
  }

  // We accumulate a new acceleration each time based on three rules
  flock(boids) {
    var sep = this.separate(boids); // Separation
    var ali = this.align(boids); // Alignment
    var coh = this.cohesion(boids); // Cohesion
    var sepPillars = this.separate(pillars); // Separation from Pillars
    // Arbitrarily weight these forces
    sep.mult(seperation);
    ali.mult(alignment);
    coh.mult(cohesion);
    sepPillars.mult(seperationPillars);
    // Add the force vectors to acceleration
    this.applyForce(sep);
    this.applyForce(ali);
    this.applyForce(coh);
    this.applyForce(sepPillars);
  }

  // Method to update position
  update() {
    // Update velocity
    this.velocity.add(this.acceleration);
    // Limit speed
    this.velocity.limit(maxspeed);
    this.position.add(this.velocity);
    // Reset accelertion to 0 each cycle
    this.acceleration.mult(0);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  seek(target) {
    var desired = p5.Vector.sub(target, this.position); // A vector pointing from the position to the target
    // Scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);

    // Steering = Desired minus Velocity
    var steer = p5.Vector.sub(desired, this.velocity);
    steer.limit(maxforce); // Limit to maximum steering force
    return steer;
  }

  render() {
    // Draw a triangle rotated in the direction of velocity
    var theta = this.velocity.heading() + radians(90);
    // heading2D() above is now heading() but leaving old syntax until Processing.js catches up

    fill(200, 100);
    stroke(255);
    push();
    translate(this.position.x, this.position.y);
    rotate(theta);
    beginShape(TRIANGLES);
    vertex(0, -rScale * 2);
    vertex(-rScale, rScale * 2);
    vertex(rScale, rScale * 2);
    endShape();
    pop();
  }

  // Wraparound
  borders() {
    if (this.position.x < -rScale) this.position.x = width + rScale;
    if (this.position.y < -rScale) this.position.y = height + rScale;
    if (this.position.x > width + rScale) this.position.x = -rScale;
    if (this.position.y > height + rScale) this.position.y = -rScale;
  }

  // Separation
  // Method checks for nearby boids and steers away
  separate(boids) {
    var desiredseparation = 25.0;
    var steer = createVector(0, 0, 0);
    var count = 0;
    // For every boid in the system, check if it's too close
    boids.forEach((other) => {
      var d = p5.Vector.dist(this.position, other.position);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if (d > 0 && d < desiredseparation) {
        // Calculate vector pointing away from neighbor
        var diff = p5.Vector.sub(this.position, other.position);
        diff.normalize();
        diff.div(d); // Weight by distance
        steer.add(diff);
        count++; // Keep track of how many
      }
    });

    // Average -- divide by how many
    if (count > 0) {
      steer.div(count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(this.velocity);
      steer.limit(maxforce);
    }
    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  align(boids) {
    var neighbordist = 50;
    var sum = createVector(0, 0);
    var count = 0;
    boids.forEach((other) => {
      var d = p5.Vector.dist(this.position, other.position);
      if (d > 0 && d < neighbordist) {
        sum.add(other.velocity);
        count++;
      }
    });

    if (count > 0) {
      sum.div(count);

      // Implement Reynolds: Steering = Desired - Velocity
      sum.normalize();
      sum.mult(maxspeed);
      var steer = p5.Vector.sub(sum, this.velocity);
      steer.limit(maxforce);
      return steer;
    } else {
      return createVector(0, 0);
    }
  }

  // Cohesion
  // For the average position (i.e. center) of all nearby boids, calculate steering vector towards that position
  cohesion(boids) {
    var neighbordist = 50;
    var sum = createVector(0, 0); // Start with empty vector to accumulate all positions
    var count = 0;
    boids.forEach((other) => {
      var d = p5.Vector.dist(this.position, other.position);
      if (d > 0 && d < neighbordist) {
        sum.add(other.position); // Add position
        count++;
      }
    });
    if (count > 0) {
      sum.div(count);
      return this.seek(sum); // Steer towards the position
    } else {
      return createVector(0, 0);
    }
  }
}

// CLASS Pillar (Repells a Boid)
class Pillar {
  constructor(x, y) {
    this.position = createVector(x, y);
  }

  render() {
    fill(200, 100);
    stroke(255);
    ellipse(this.position.x, this.position.y, 5);
  }
}
