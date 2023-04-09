/*
 * huansic_malloc.h
 *
 *  Created on: Oct 19, 2022
 *      Author: Zonghuan Wu
 */

#ifndef INC_HUANSIC_MALLOC_H_
#define INC_HUANSIC_MALLOC_H_

#include "huansic_types.h"
#include <math.h>

void huansic_path_malloc_init();

Path* huansic_path_new();

void huansic_path_delete(Path *ptr);

void huansic_path_cascade(Path *last, Path *next);

Path* huansic_path_break(Path *newTail);


void huansic_order_init();

Order* huansic_order_new(int8_t id);

void huansic_order_delete(Order *ptr);

// find in the order pool the order whose start location is the closest
Order* huansic_order_findClosestStart(Coordinate *coord);

// find in the delivery list the order whose destination location is the closest
Order* huansic_order_findClosestDest(Coordinate *coord);

#endif /* INC_HUANSIC_MALLOC_H_ */
