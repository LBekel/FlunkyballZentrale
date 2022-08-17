/*
 * game.c
 *
 *  Created on: 02.04.2022
 *      Author: LBekel
 */
#include "FreeRTOS.h"
#include "task.h"
#include "app_log.h"
#include "app_assert.h"
#include "app.h"
#include "game.h"


#ifndef GAME_TASK_STACK_SIZE
#define GAME_TASK_STACK_SIZE      configMINIMAL_STACK_SIZE
#endif

#ifndef GAME_TASK_PRIO
#define GAME_TASK_PRIO      21
#endif

#define MAXPLAYER           2
#define BOTTLE_FULL         0.05
#define BOTTLE_EMPTY_UPPER  0.02
#define BOTTLE_EMPTY_LOWER  0.018
#define BOTTLE_NO           0.001


static void game_task(void *arg);
void send_led_to_team(uint8_t freq, uint8_t team);
static player_t players[MAXPLAYER];

typedef enum
{
    wait_for_players,
    check_all_scales,
    send_start_team,
    check_central_bottle_falling,
    send_release_team,
    check_liftup_team,
    check_central_bottle_standing,
    check_buzzer_team,
    send_unrelease_team,
    check_putdown_team,
    check_beer_empty,
    send_lose_team,
    send_win_team
} game_state_t;

void game_init(void)
{
  TaskHandle_t xHandle = NULL;
  static StaticTask_t xTaskBuffer;
  static StackType_t  xStack[GAME_TASK_STACK_SIZE];

  app_log_info("start game task\n\r");
  // Create Blink Task without using any dynamic memory allocation
  xHandle = xTaskCreateStatic(game_task,
                              "game task",
                              GAME_TASK_STACK_SIZE,
                              ( void * ) NULL,
                              tskIDLE_PRIORITY + 1,
                              xStack,
                              &xTaskBuffer);

  // Since puxStackBuffer and pxTaskBuffer parameters are not NULL,
  // it is impossible for xHandle to be null. This check is for
  // rigorous example demonstration.
  EFM_ASSERT(xHandle != NULL);
}

/*******************************************************************************
 * Game task.
 ******************************************************************************/
static void game_task(void *arg)
{
    (void) &arg;
    game_state_t gamestate = wait_for_players;
    uint8_t activeteam = 1;

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        for(uint8_t var = 0; var < MAXPLAYER; var++)
        {
            get_player_data(&players[var], var);
            if(players[var].team == 1)
            {
                app_log_append("T:%d W:%6.3fkg ",players[var].team,  players[var].weight);
            }
        }
        for(uint8_t var = 0; var < MAXPLAYER; var++)
        {
            get_player_data(&players[var], var);
            if(players[var].team == 2)
            {
                app_log_append("T:%d W:%6.3fkg ",players[var].team,  players[var].weight);
            }
        }
        //app_log_append("\n\r");
        bool readytostart = true;
        switch(gamestate)
        {
            case wait_for_players:
                app_log_info("wait_for_players\r\n");
                if((get_team_count(1)==MAXPLAYER/2)&&(get_team_count(2)==MAXPLAYER/2))
                {
                    gamestate = check_all_scales;
                }
                break;
            case check_all_scales:
                app_log_info("check_all_scales\r\n");

                readytostart = true;
                for(uint8_t var = 0; var < MAXPLAYER; var++)
                {
                    readytostart = readytostart && players[var].weight > BOTTLE_FULL;
                }
                if(readytostart)
                {
                    gamestate = send_start_team;
                }
                break;

            case send_start_team:
                app_log_info("send_start_team %d\r\n",activeteam);
                if(activeteam == 1)
                {
                    send_led_to_team(10,1);
                    send_led_to_team(0,2);
                }
                else
                {
                    send_led_to_team(0,1);
                    send_led_to_team(10,2);
                }
                gamestate = check_central_bottle_falling;
                break;

            case check_central_bottle_falling:
                app_log_info("check_central_bottle falling %d\r\n",activeteam);
                if(getbuttonstate())
                {
                    //Flasche gefallen
                    gamestate = send_release_team;
                }

                break;

            case send_release_team:
                app_log_info("send_release_team %d\r\n",activeteam);
                for(uint8_t var = 0; var < MAXPLAYER; var++)
                {
                    send_led_to_team(20,activeteam);
                }
                gamestate = check_liftup_team;
                break;

            case check_liftup_team:
                app_log_info("check liftup team %d\r\n",activeteam);
                bool allliftup = true;
                for(uint8_t var = 0; var < MAXPLAYER; var++)
                {
                    get_player_data(&players[var], var);
                    if(players[var].team == activeteam)
                    {
                        allliftup = allliftup && players[var].weight < BOTTLE_NO;
                    }
                }
                if(allliftup)
                {
                    gamestate = check_central_bottle_standing;
                }

                break;

            case check_central_bottle_standing:
                app_log_info("check_central_bottle standing %d\r\n",activeteam);
                if(getbuttonstate())
                {
                    //Flasche steht wieder
                    gamestate = send_unrelease_team;
                }
                break;
            case send_unrelease_team:
                app_log_info("send unrelease team %d\r\n",activeteam);
                send_led_to_team(10,activeteam);
                gamestate = check_putdown_team;
                break;

            case check_putdown_team:
                app_log_info("check_put down_team %d\r\n",activeteam);
                readytostart = true;
                for(uint8_t var = 0; var < MAXPLAYER; var++)
                {
                    if(players[var].team == activeteam)
                    {
                        readytostart = readytostart && players[var].weight >= BOTTLE_EMPTY_LOWER;
                    }
                }
                if(readytostart)
                {
                    gamestate = check_beer_empty;
                }
                break;
            case check_beer_empty:
                app_log_info("check_beer_empty %d\r\n",activeteam);

                bool empty = true;
                for(uint8_t var = 0; var < MAXPLAYER; var++)
                {
                    if(players[var].team == activeteam)
                    {
                        empty = empty && players[var].weight <= BOTTLE_EMPTY_UPPER;
                    }
                }
                if(empty)
                {
                    gamestate = send_win_team;
                }
                else
                {
                    if(activeteam == 1)
                    {
                        activeteam = 2;
                    }
                    else
                    {
                        activeteam = 1;
                    }
                    gamestate = send_start_team;
                }
                break;
            case send_win_team:
                app_log_info("send_win_team %d\r\n",activeteam);

                break;
            default:
                break;

        }
    }

}
void send_led_to_team(uint8_t freq, uint8_t team)
{
    for(uint8_t var = 0; var < MAXPLAYER; var++)
    {
        if(players[var].team == team)
        {
            send_led_frequency(freq, var);
        }
    }
}

