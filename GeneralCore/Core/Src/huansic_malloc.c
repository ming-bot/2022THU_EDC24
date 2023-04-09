/*
 * huansic_malloc.c
 *
 *  Created on: Oct 19, 2022
 *      Author: Zonghuan Wu
 */

#include "huansic_malloc.h"

#ifndef HUANSIC_PATH_PREMALLOC_SIZE
#define HUANSIC_PATH_PREMALLOC_SIZE 16
#endif

#ifndef HUANSIC_ORDER_PREMALLOC_SIZE
#define HUANSIC_ORDER_PREMALLOC_SIZE 100
#endif

Path pathBuffers[HUANSIC_PATH_PREMALLOC_SIZE];
Order orderBuffers[HUANSIC_ORDER_PREMALLOC_SIZE];

extern Order *delivering[8];

__weak void custom_path_free_fault(Path *ptr) {

}

__weak void custom_order_free_fault(Order *ptr) {

}

void huansic_path_malloc_init() {
	uint8_t i;
	for (i = 0; i < HUANSIC_PATH_PREMALLOC_SIZE; i++) {
		pathBuffers[i].referenceCount = 0;	// clear referenced counter so that it can be malloced
		pathBuffers[i].nextPath = 0;		// set to nullptr just in case
	}
}

Path* huansic_path_new() {
	Path *retPtr = 0;
	uint8_t i;
	for (i = 0; i < HUANSIC_PATH_PREMALLOC_SIZE; i++) {
		if (pathBuffers[i].referenceCount == 0) {
			pathBuffers[i].referenceCount = 1;
			retPtr = &pathBuffers[i];
			break;		// could've just returned the pointer, but to keep it consistent...
		}
	}

	return retPtr;
}

void huansic_path_delete(Path *ptr) {
	uint8_t i;
	for (i = 0; i < HUANSIC_PATH_PREMALLOC_SIZE; i++) {
		if (pathBuffers[i].nextPath == ptr) {			// clear all references in the list
			ptr->referenceCount--;
			pathBuffers[i].nextPath = 0;
		}
	}

	if (ptr->referenceCount != 1) {
		custom_path_free_fault(ptr);		// if there're other references, run the ISR
	}

	// and free itself
	ptr->referenceCount = 0;
}

void huansic_path_cascade(Path *last, Path *next) {
	if (!last)		// make "next" head; no actions
		return;

	if (last->nextPath != next) {
		if (!last->nextPath)
			last->nextPath->referenceCount--;
		last->nextPath = next;

		if (next)		// if next is non-zero, make sure to increase the reference count
			next->referenceCount++;
	}
}

Path* huansic_path_break(Path *newTail) {
	if (!newTail)
		return 0;

	Path *retPath = newTail->nextPath;		// return the new head (if any)
	if (retPath) {
		retPath->referenceCount--;
		newTail->nextPath = 0;
	}
	return retPath;
}

void huansic_order_init() {
	uint8_t i;
	for (i = 0; i < HUANSIC_ORDER_PREMALLOC_SIZE; i++){
		orderBuffers[i].id = -1;
		orderBuffers[i].startTime = 0;
	}
	for (i = 0; i < 5; i++)
		delivering[i] = &orderBuffers[i];		// give it some default value
}

Order* huansic_order_new(int8_t id) {
	if (id == -1)
		return 0;

	uint8_t i;

	// find duplicates
	for (i = 0; i < HUANSIC_ORDER_PREMALLOC_SIZE; i++) {
		if (orderBuffers[i].id == id)
			return &orderBuffers[i];
	}

	// find spares
	for (i = 0; i < HUANSIC_ORDER_PREMALLOC_SIZE; i++) {
		if (orderBuffers[i].id == -1) {
			orderBuffers[i].id = id;
			return &orderBuffers[i];
		}
	}

	return 0;
}

void huansic_order_delete(Order *ptr) {
	if (ptr->id == -1)
		custom_order_free_fault(ptr);
	else{
		ptr->id = -1;		// simple as is
		ptr->startTime = 0;
	}
}

Order* huansic_order_findClosestDest(Coordinate *coord) {
	Order *retOrder = 0;
	float currentDistance, closestDistance = 1000000000000;
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (delivering[i]->id == -1)		// if it is an empty order
			continue;
		currentDistance = powf(delivering[i]->destCoord.x - coord->x, 2)
				+ powf(delivering[i]->destCoord.y - coord->y, 2);
		if (currentDistance < closestDistance) {
			closestDistance = currentDistance;
			retOrder = delivering[i];
		}
	}
	return retOrder;
}

Order* huansic_order_findClosestStart(Coordinate *coord) {
	Order *retOrder = 0;
	float currentDistance, closestDistance = 1000000000000;
	uint8_t i;
	for (i = 0; i < HUANSIC_ORDER_PREMALLOC_SIZE; i++) {
		if (orderBuffers[i].id == -1)		// if it is an empty order
			continue;
		if (&orderBuffers[i] == delivering[0] || &orderBuffers[i] == delivering[1]
				|| &orderBuffers[i] == delivering[2] || &orderBuffers[i] == delivering[3]
				|| &orderBuffers[i] == delivering[4])		// if it is in delivery list
			continue;

		currentDistance = powf(orderBuffers[i].startCoord.x - coord->x, 2)
				+ powf(orderBuffers[i].startCoord.y - coord->y, 2);
		if (currentDistance < closestDistance) {
			closestDistance = currentDistance;
			retOrder = &orderBuffers[i];
		}
	}
	return retOrder;
}
