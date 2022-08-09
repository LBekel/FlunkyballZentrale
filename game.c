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


#ifndef GAME_TASK_STACK_SIZE
#define GAME_TASK_STACK_SIZE      configMINIMAL_STACK_SIZE
#endif

#ifndef GAME_TASK_PRIO
#define GAME_TASK_PRIO            21
#endif

#define MAXPLAYER 2

static void game_task(void *arg);

typedef enum
{
    check_all_scales,
    send_start_team,
    check_central_bottle,
    send_release_team,
    check_buzzer_team,
    send_unrelease_team,
    check_scales_team,
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
    game_state_t gamestate = check_all_scales;
    static conn_properties_t conn_properties[MAXPLAYER];
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        switch(gamestate)
        {
            case check_all_scales:
                //app_log_info("check_all_scales\r\n");
                ;
                bool readytostart = true;
                app_log_append("Weight: ");
                for(uint8_t var = 0; var < MAXPLAYER; var++)
                {
                    get_player_data(&conn_properties[var], var);
                    readytostart = readytostart && conn_properties[var].weight > 0.3;
                    app_log_append("%d: %6.3fkg ",var,  conn_properties[var].weight);
                }
                app_log_append("\n\r");
                if(readytostart)
                {
                    gamestate = send_start_team;
                }

                break;

            case send_start_team:
                app_log_info("send_start_team\r\n");
                gamestate = check_central_bottle;
                break;

            case check_central_bottle:
                app_log_info("check_central_bottle\r\n");
                if(getbuttonstate())
                {
                    //Flasche gefallen
                    gamestate = send_release_team;
                }

                break;

            case send_release_team:
                app_log_info("send_release_team\r\n");
                break;
            default:
                break;

        }
    }

}
