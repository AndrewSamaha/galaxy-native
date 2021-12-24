/*
 * galaxy.c
 * 		by Andrew Samaha
 *		based heavily on lavanet.c by Robert 'Bobby' Zenz
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <X11/Xlib.h>

#include "vroot.h"

#define FALSE 0
#define TRUE 1
#define BLACK 0x000000
#define WHITE 0xFFFFFF
#define GRAVITY 0.001
#define FRICTION 0
#define FRICTION_STATIC 1
#define FRICTION_AIR 0
#define RESTITUTION 0.5
#define POINT_MASS 0.001
#define DEBUG 0
#define STARTING_VELOCITY_COEFF .0001

struct vector {
	float x;
	float y;
};

struct line {
	float startX;
	float startY;
	float endX;
	float endY;
	int value;
};

struct body {
	struct vector position;
	struct vector velocity;
	struct vector force;
	float mass;
};

int targetRed = 0;
int targetGreen = 255;
int targetBlue = 255;

// Global, sorry.
int debug = FALSE;
int pointCount = 200;
int bodyCount = 200;

float minimumDistance = 100;
int targetFps = 60;
float topChange = 0.1f;
float topSpeed = 0.5f;

float get_random() {
	return ((float) rand() / RAND_MAX - 0.5f) * 2;
}
float randPositiveFloat(){
	return ((float) rand() / RAND_MAX);
}

long getCurrentTime() {
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	long currentTimeLong = currentTime.tv_sec * 1000000 + currentTime.tv_usec;
	return currentTimeLong;
}

ulong make_color(u_char red, u_char green, u_char blue) {
	ulong color = red;
	color = color << 8;

	color |= green;
	color = color << 8;

	color |= blue;

	return color;
}

float sign(float x) {
	float val = x > 0;
	return val - (x < 0);
}

void draw_lines(struct line *lines, int lineCount, Display *dpy, GC g,
		Pixmap pixmap) {
	int idx;
	for (idx = 0; idx < lineCount; idx++) {
		struct line *line = &lines[idx];
		XSetForeground(dpy, g, make_color(targetRed * line->value, targetGreen
				* line->value, targetBlue * line->value));
		XDrawLine(dpy, pixmap, g, line->startX, line->startY, line->endX,
				line->endY);
	}
}

int gather_lines(struct vector *points, struct line **lines) {
	int counter = 0;
	int idx;
	for (idx = 0; idx < pointCount; idx++) {
		struct vector *pointA = &points[idx];
		int idx2 = 0;
		for (idx2 = idx + 1; idx2 < pointCount; idx2++) {
			struct vector *pointB = &points[idx2];

			// Check distance between points
			int distanceX = abs(pointA->x - pointB->x);
			int distanceY = abs(pointA->y - pointB->y);

			double distance = sqrt(pow(distanceX, 2) + pow(distanceY, 2));

			if (distance < minimumDistance) {
				counter++;

				*lines = realloc(*lines, counter * sizeof(struct line));
				struct line *newLine = &(*lines)[counter - 1];
				newLine->startX = pointA->x;
				newLine->startY = pointA->y;
				newLine->endX = pointB->x;
				newLine->endY = pointB->y;
				newLine->value = (int) floor(distance / minimumDistance * 255);
			}
		}
	}

	return counter;
}

int sort_lines(const void *a, const void *b) {
	struct line *lineA = (struct line*) a;
	struct line *lineB = (struct line*) b;
	if (lineA->value == lineB->value) {
		return 0;
	} else if (lineA->value > lineB->value) {
		return -1;
	}

	return 1;
}
// int gather_lines(struct vector *points, struct line **lines) {
void createBodies(XWindowAttributes wa, struct body **bodies) {
	for (int i = 0; i < bodyCount; i++) {
		//*lines  = realloc(*lines , counter * sizeof(struct line));
		*bodies = realloc(*bodies, (i+1) * sizeof(struct body));
		struct body *newbody = &(*bodies)[i];
		newbody->position.x = randPositiveFloat() * wa.width;
		newbody->position.y = randPositiveFloat() * wa.height;
		newbody->velocity.x = (randPositiveFloat() - .5) * STARTING_VELOCITY_COEFF;
		newbody->velocity.y = (randPositiveFloat() - .5) * STARTING_VELOCITY_COEFF;
		newbody->force.x = 0;
		newbody->force.y = 0;
		newbody->mass = POINT_MASS;
		if (DEBUG) {
			printf("creating %d: POS %f,  %f    VEL %f,  %f\n", i, newbody->position.x, newbody->position.y, newbody->velocity.x, newbody->velocity.y);
		}
	}
}

void calcForces(struct body *bodies, long timeDelta) {
	if (DEBUG) printf("timeDelta %ld\n", timeDelta);
	if (timeDelta > 1000000) return;
	for (int i = 0; i < bodyCount; i++) {
		if ( DEBUG && i == 1 ) {
			printf("calcForces   %d:  startPos  %f,  %f\n", i, bodies[i].position.x, bodies[i].position.y);
		}
		for (int j = 0; j < bodyCount; j++) {
			if (i == j) continue;
			
			float Dx = bodies[j].position.x - bodies[i].position.x;
			float Dy = bodies[j].position.y - bodies[i].position.y;
			float force = timeDelta * GRAVITY * bodies[j].mass * bodies[i].mass / sqrt(Dx * Dx + Dy * Dy);
			float angle = atan2f(Dy, Dx);
			bodies[i].force.x += force * cosf(angle);
			bodies[i].force.y += force * sinf(angle);
		}
	}
}
void applyForces(struct body *bodies) {
	for (int i = 0; i < bodyCount; i++) {
		bodies[i].velocity.x += bodies[i].force.x;
		bodies[i].velocity.y += bodies[i].force.y;
		if ( DEBUG && i == 1 ) {
			printf("applyForces  %d:  force %f,  %f\n", i, bodies[i].force.x, bodies[i].force.y);
		}
	}	
}
void moveVelocity(struct body *bodies, long timeDelta) {
	for (int i = 0; i < bodyCount; i++) {
		if ( DEBUG && i == 1 ) {
			printf("moveVelocity %d:  startPos  %f,  %f\n", i, bodies[i].position.x, bodies[i].position.y);
		}
		bodies[i].position.x += bodies[i].velocity.x * timeDelta;
		bodies[i].position.y += bodies[i].velocity.y * timeDelta;
		if ( DEBUG && i == 1 ) {
			printf("moveVelocity %d:  stopEnd   %f,  %f\n", i, bodies[i].position.x, bodies[i].position.y);
		}
	}
}

void drawBodies(Display *dpy, GC g, Pixmap pixmap, struct body *bodies) {
	for (int i = 0; i < bodyCount; i++) {
		XSetForeground(
			dpy, 
			g, 
			make_color(
				255, 
				255, 
				255));
		XDrawPoint(
			dpy, pixmap, g,
			bodies[i].position.x,
			bodies[i].position.y);
			if ( DEBUG && i == 1 ) {
				printf("drawBodies    %d:  %f,  %f\n", i, bodies[i].position.x, bodies[i].position.y);
			}
		bodies[i].force.x = 0;
		bodies[i].force.y = 0;
	}	

}

void move_points(struct vector *points, struct vector *velocities,
		XWindowAttributes wa) {
	int idx;
	for (idx = 0; idx < pointCount; idx++) {
		struct vector *velocity = &velocities[idx];

		velocity->x += get_random() * topChange;
		velocity->y += get_random() * topChange;

		if (abs(velocity->x) > topSpeed) {
			velocity->x = topSpeed * sign(velocity->x);
		}

		if (abs(velocity->y) > topSpeed) {
			velocity->y = topSpeed * sign(velocity->y);
		}

		struct vector *point = &points[idx];
		point->x += velocity->x;
		point->y += velocity->y;

		if (point->x < 0) {
			point->x = wa.width;
		}
		if (point->x > wa.width) {
			point->x = 0;
		}
		if (point->y < 0) {
			point->y = wa.height;
		}
		if (point->y > wa.height) {
			point->y = 0;
		}
	}
}

void parse_arguments(int argc, char *argv[]) {
	int idx;
	for (idx = 1; idx < argc; idx++) {
		if (strcasecmp(argv[idx], "--debug") == 0) {
			debug = TRUE;
		}
	}
}

int main(int argc, char *argv[]) 
{
	parse_arguments(argc, argv);

	// Some stuff
	int sleepFor = (int) 1000 / targetFps * 1000;

	// Create our display
	Display *dpy = XOpenDisplay(getenv("DISPLAY"));

	char *xwin = getenv ("XSCREENSAVER_WINDOW");

	int root_window_id = 0;

  	if (xwin)
  	{
    	root_window_id = strtol (xwin, NULL, 0);
  	}

	// Get the root window
	Window root;
	if (debug == FALSE) {
		// Get the root window
		// root = DefaultRootWindow(dpy);
		if (root_window_id == 0)
    	{
		   // root = DefaultRootWindow(dpy);
      	   printf ("usage as standalone app: %s --debug\n", argv[0]);
      		return EXIT_FAILURE;
    	}
    	else
    	{
      		root = root_window_id;
    	}
	} else {
		// Let's create our own window.
		int screen = DefaultScreen(dpy);
		root = XCreateSimpleWindow(dpy, RootWindow(dpy, screen), 24, 48, 860,
				640, 1, BlackPixel(dpy, screen), WhitePixel(dpy, screen));
		XMapWindow(dpy, root);
	}

	XSelectInput (dpy, root, ExposureMask | StructureNotifyMask);

	// Get the window attributes
	XWindowAttributes wa;
	XGetWindowAttributes(dpy, root, &wa);

	// Create the buffer
	Pixmap double_buffer = XCreatePixmap(dpy, root, wa.width, wa.height,
			wa.depth);

	// And new create our graphics context to draw on
	GC g = XCreateGC(dpy, root, 0, NULL);

	struct timeval tv;
	gettimeofday(&tv, NULL);
	srand(tv.tv_sec);

	struct body *bodies = malloc(sizeof(struct body));
	//struct body bodies[bodyCount];
	struct vector points[pointCount];
	struct vector velocities[pointCount];
	int counter = 0;

	long lastDrawTime = getCurrentTime();

	for (counter = 0; counter < pointCount; counter++) {
		points[counter].x = rand() % wa.width;
		points[counter].y = rand() % wa.height;

		velocities[counter].x = get_random() * topSpeed;
		velocities[counter].y = get_random() * topSpeed;
	}


	// this is to terminate nicely:
	Atom wmDeleteMessage = XInternAtom(dpy, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(dpy, root, &wmDeleteMessage, 1);

/// int lineCount = gather_lines(points, &lines);
	createBodies(wa, &bodies);

	while ( TRUE )
	{
		XEvent event;
		if (XCheckWindowEvent(dpy, root, StructureNotifyMask, &event) ||
		    XCheckTypedWindowEvent (dpy, root, ClientMessage, &event)) // needed to catch ClientMessage
		{
			if (event.type == ConfigureNotify) 
        	{
          		XConfigureEvent xce = event.xconfigure;

		        // This event type is generated for a variety of
          		// happenings, so check whether the window has been
          		// resized.

          		if (xce.width != wa.width || xce.height != wa.height) 
          		{
            		wa.width = xce.width;
            		wa.height = xce.height;

    				XFreePixmap(dpy, double_buffer);
    				double_buffer = XCreatePixmap(dpy, root, wa.width, wa.height,
							wa.depth);
          
            		continue;
          		}
        	}
			else if (event.type == ClientMessage)
        	{
            	if (event.xclient.data.l[0] == wmDeleteMessage)
            	{
                	break;
            	}
			}
		}

		// Clear the pixmap used for double buffering
		XSetBackground(dpy, g, BLACK);
		XSetForeground(dpy, g, BLACK);
		XFillRectangle(dpy, double_buffer, g, 0, 0, wa.width, wa.height);

		// Move the points around
		move_points(points, velocities, wa);

		// Gather the lines and draw them
		struct line *lines = malloc(sizeof(struct line));
		int lineCount = gather_lines(points, &lines);
		qsort(lines, lineCount, sizeof(struct line), sort_lines);
		
		long currentTime = getCurrentTime();
		long delta = currentTime - lastDrawTime;
		calcForces(bodies, delta);
		applyForces(bodies);
		moveVelocity(bodies, delta);
		drawBodies(dpy, g, double_buffer, bodies);
		// draw_lines(lines, lineCount, dpy, g, double_buffer);
		
		lastDrawTime = getCurrentTime();

		free(lines);

		XCopyArea(dpy, double_buffer, root, g, 0, 0, wa.width, wa.height, 0, 0);
		XFlush(dpy);

		usleep(sleepFor);
		
		
	}

	// cleanup
	XFreePixmap(dpy, double_buffer);
	XFreeGC (dpy, g);
  	XDestroyWindow(dpy, root);
  	XCloseDisplay (dpy);

	return EXIT_SUCCESS;
}
