/*
 * lavanet.c
 *
 *      Author: Robert 'Bobby' Zenz
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <X11/Xlib.h>

#include "vroot.h"

#define FALSE 0
#define TRUE 1
#define BLACK 0x000000
#define WHITE 0xFFFFFF

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

int targetRed = 0;
int targetGreen = 255;
int targetBlue = 255;

// Global, sorry.
int debug = FALSE;
int pointCount = 200;
float minimumDistance = 100;
int targetFps = (int) 1000 / 60 * 1000;
float topChange = 0.1f;
float topSpeed = 0.5f;

float get_random() {
	return ((float) rand() / RAND_MAX - 0.5f) * 2;
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
		XSetForeground(dpy, g, make_color(targetRed * line->value, targetGreen * line->value, targetBlue * line->value));
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

int main(int argc, char *argv[]) {
	parse_arguments(argc, argv);

	// Create our display
	Display *dpy = XOpenDisplay(getenv("DISPLAY"));

	// Get the root window
	Window root;
	if (debug == FALSE) {
		// Get the root window
		root = DefaultRootWindow(dpy);
	} else {
		// Let's create our own window.
		int screen = DefaultScreen(dpy);
		root = XCreateSimpleWindow(dpy, RootWindow(dpy, screen), 24, 48, 860,
				640, 1, BlackPixel(dpy, screen), WhitePixel(dpy, screen));
		XMapWindow(dpy, root);
	}

	// Get the window attributes
	XWindowAttributes wa;
	XGetWindowAttributes(dpy, root, &wa);

	// Create the buffer
	Pixmap double_buffer = XCreatePixmap(dpy, root, wa.width, wa.height,
			wa.depth);

	// And new create our graphics context to draw on
	GC g = XCreateGC(dpy, root, 0, NULL);

	srand(0);

	struct vector points[pointCount];
	struct vector velocities[pointCount];
	int counter = 0;
	for (counter = 0; counter < pointCount; counter++) {
		points[counter].x = rand() % wa.width;
		points[counter].y = rand() % wa.height;

		velocities[counter].x = get_random() * topSpeed;
		velocities[counter].y = get_random() * topSpeed;
	}

	// I hate this...
	while (1) {
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
		draw_lines(lines, lineCount, dpy, g, double_buffer);
		free(lines);

		XCopyArea(dpy, double_buffer, root, g, 0, 0, wa.width, wa.height, 0, 0);
		XFlush(dpy);

		// This fucking function takes microseconds! MICRO!
		usleep(targetFps);
	}

	return 0;
}
