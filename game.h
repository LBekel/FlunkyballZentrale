/*
 * game.h
 *
 *  Created on: 02.04.2022
 *      Author: LBekel
 */

#ifndef GAME_H_
#define GAME_H_

void game_init(void);

typedef struct
{
    uint8_t team;
    float weight;
} player_t;

#endif /* GAME_H_ */
