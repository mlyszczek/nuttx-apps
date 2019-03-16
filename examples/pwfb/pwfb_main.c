/****************************************************************************
 * examples/pwfb/pwfb_main.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/boardctl.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <pthread.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#ifdef CONFIG_VNCSERVER
#  include <nuttx/video/vnc.h>
#endif

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxfonts.h>

#include "pwfb_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* If not specified, assume that the hardware supports one video plane */

#ifndef CONFIG_EXAMPLES_PWFB_VPLANE
#  define CONFIG_EXAMPLES_PWFB_VPLANE 0
#endif

/* If not specified, assume that the hardware supports one LCD device */

#ifndef CONFIG_EXAMPLES_PWFB_DEVNO
#  define CONFIG_EXAMPLES_PWFB_DEVNO 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_exitcode = NXEXIT_SUCCESS;

static const uint8_t g_wndomsg1[] = "NuttX is cool!";
static const uint8_t g_wndomsg2[] = "NuttX is fun!";
static const uint8_t g_wndomsg3[] = "NuttX is groovy!";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_state_initialize
 ****************************************************************************/

static void nxeg_state_initialize(FAR struct nxeg_state_s *st)
{
  FAR const struct nx_font_s *fontset;

  /* Initialize semaphores */

  sem_init(&st->semevent, 0, 0);

  /* Initialize color information */

  st->wndo[0].color[0] = CONFIG_EXAMPLES_PWFB_COLOR1;
  st->wndo[1].color[0] = CONFIG_EXAMPLES_PWFB_COLOR2;
  st->wndo[2].color[0] = CONFIG_EXAMPLES_PWFB_COLOR3;

  /* Connect to the font cache */

  st->fcache = nxf_cache_connect(CONFIG_EXAMPLES_PWFB_FONTID,
                                 nxgl_mxpixel_t fgcolor,
                                 nxgl_mxpixel_t bgcolor,
                                 int bpp, int maxglyph);
  if (st->fcache == NULL)
    {
      printf("ERROR: Failed to connect to font cache for font ID %d: %d\n",
             CONFIG_EXAMPLES_PWFB_FONTID, errno);
      return;
    }

  /* Get the handle of the font managed by the font cache */

  st->hfont = nxf_cache_getfonthandle(st->fcache);
  if (st->hfont == NULL)
    {
      printf("ERROR: Failed to get handle for font ID %d: %d\n",
           CONFIG_EXAMPLES_PWFB_FONTID, errno);
      return;
    }

  /* Get information about the font set being used and save this in the
   * state structure
   */

  fontset      = nxf_getfontset(st->hfont);
  st->fheight  = fontset->mxheight;
  st->fwidth   = fontset->mxwidth;
  st->spwidth  = fontset->spwidth;
}

/****************************************************************************
 * Name: nxeg_server_initialize
 ****************************************************************************/

static int nxeg_server_initialize(FAR struct nxeg_state_s wstate *st)
{
  struct sched_param param;
  pthread_t thread;
  int ret;
  int i;

  /* Set the client task priority */

  param.sched_priority = CONFIG_EXAMPLES_PWFB_CLIENTPRIO;
  ret = sched_setparam(0, &param);
  if (ret < 0)
    {
      printf("nxeg_server_initialize: sched_setparam failed: %d\n" , ret);
      g_exitcode = NXEXIT_SCHEDSETPARAM;
      return ERROR;
    }

  /* Start the NX server kernel thread */

  ret = boardctl(BOARDIOC_NX_START, 0);
  if (ret < 0)
    {
      printf("nxeg_server_initialize: Failed to start the NX server: %d\n", errno);
      g_exitcode = NXEXIT_TASKCREATE;
      return ERROR;
    }

  /* Connect to the server */

  st->hnx = nx_connect();
  if (st->hnx)
    {
       pthread_attr_t attr;

#ifdef CONFIG_VNCSERVER
      /* Setup the VNC server to support keyboard/mouse inputs */

      ret = vnc_default_fbinitialize(0, st->hnx);
      if (ret < 0)
        {
          printf("vnc_default_fbinitialize failed: %d\n", ret);
          nx_disconnect(st->hnx);
          g_exitcode = NXEXIT_FBINITIALIZE;
          return ERROR;
        }
#endif
       /* Start a separate thread to listen for server events.  This is probably
        * the least efficient way to do this, but it makes this example flow more
        * smoothly.
        */

       (void)pthread_attr_init(&attr);
       param.sched_priority = CONFIG_EXAMPLES_PWFB_LISTENERPRIO;
       (void)pthread_attr_setschedparam(&attr, &param);
       (void)pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_PWFB_STACKSIZE);

       ret = pthread_create(&thread, &attr, nx_listenerthread, NULL);
       if (ret != 0)
         {
            printf("nxeg_server_initialize: pthread_create failed: %d\n", ret);
            g_exitcode = NXEXIT_PTHREADCREATE;
            return ERROR;
         }

       /* Don't return until we are connected to the server */

       while (!st->connected)
         {
           /* Wait for the listener thread to wake us up when we really
            * are connected.
            */

           (void)sem_wait(&st->semevent);
         }
    }
  else
    {
      printf("nxeg_server_initialize: nx_connect failed: %d\n", errno);
      g_exitcode = NXEXIT_NXCONNECT;
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwfb_main
 ****************************************************************************/

#ifdef BUILD_MODULE
int main(int argc, FAR char *argv[])
#else
int pwfb_main(int argc, char *argv[])
#endif
{
  struct nxeg_state_s wstate;
  struct nxgl_size_s size;
  struct nxgl_point_s pt;
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize */

  memset(st, 0, sizeof(struct nxeg_state_s));
  ret = nxeg_server_initialize(&wstate);

  printf("pwfb_main: NX handle=%p\n", wstate.hnx);

  if (wstate.hnx == NULL || ret < 0)
    {
      printf("pwfb_main: Failed to get NX handle: %d\n", errno);

      g_exitcode = NXEXIT_NXOPEN;
      goto errout;
    }

  /* Initialize the window state */

  nxeg_state_initialize(&wstate);

  /* Set the background to the configured background color */

  printf("pwfb_main: Set background color=%d\n",
         CONFIG_EXAMPLES_PWFB_BGCOLOR);

  color = CONFIG_EXAMPLES_PWFB_BGCOLOR;
  ret = nx_setbgcolor(wstate.hnx, &color);
  if (ret < 0)
    {
      printf("pwfb_main: nx_setbgcolor failed: %d\n", errno);

      g_exitcode = NXEXIT_NXSETBGCOLOR;
      goto errout_with_nx;
    }

  /* Create window #1 */

  printf("pwfb_main: Create window #1\n");

  wstate.wndo[0].hwnd = nxtk_openwindow(wstate.hnx, NXBE_WINDOW_RAMBACKED,
                                        &g_nxcb, (FAR void *)&wstate);
  if (wstate.wndo[0].hwnd == NULL)
    {
      printf("pwfb_main: ERROR: nxtk_openwindow failed: %d\n", errno);
      goto errout_with_nx;
    }

  printf("pwfb_main: hwnd1=%p\n", wstate.wndo[0].hwnd);

  /* Wait until we have the screen resolution */

  while (!wstate.haveres)
    {
      (void)sem_wait(&wstate.semevent);
    }

  printf("pwfb_main: Screen resolution (%d,%d)\n",
         wstate.xres, wstate.yres);

  /* Set the size of the window #1 */

  size.w = wstate.xres / 2;
  size.h = wstate.yres / 2;

  printf("pwfb_main: Set window #1 size to (%d,%d)\n", size.w, size.h);

  ret = nx_setsize(wstate.wndo[0].hwnd, &size);
  if (ret < 0)
    {
      printf("pwfb_main: nx_setsize failed: %d\n", errno);
      goto errout_with_hwnd1;
    }

  /* Set the position of window #1 */

  pt.x = wstate.xres / 8;
  pt.y = wstate.yres / 8;

  printf("pwfb_main: Set window #1 position to (%d,%d)\n", pt.x, pt.y);

  ret = nxtk_setposition(wstate.wndo[0].hwnd, &pt);
  if (ret < 0)
    {
      printf("pwfb_main: nxtk_setposition failed: %d\n", errno);
      goto errout_with_hwnd1;
    }

  /* Set up for motion */

  wstate.wndo[0].xmax   = itob32(wstate.xres - size.w - 1);
  wstate.wndo[0].ymax   = itob32(wstate.yres - size.h - 1);
  wstate.wndo[0].ypos   = itob32(pt.y);
  wstate.wndo[0].xpos   = itob32(pt.x);
  wstate.wndo[0].ypos   = itob32(pt.y);
  wstate.wndo[0].deltax = dtob32(1.58)
  wstate.wndo[0].deltay = dtob32(4.5)

  /* Create window #2 */

  printf("pwfb_main: Create window #2\n");

  wstate.wndo[1].hwnd = nxtk_openwindow(wstate.hnx, NXBE_WINDOW_RAMBACKED,
                                        &g_nxcb, (FAR void *)&wstate);
  if (wstate.wndo[1].hwnd == NULL)
    {
      printf("pwfb_main: ERROR: nxtk_openwindow failed: %d\n", errno);
      goto errout_with_hwnd1;
    }

  printf("pwfb_main: hwnd2=%p\n", wstate.wndo[1].hwnd);

  /* Set the size of the window #2 == size of window 1*/

  printf("pwfb_main: Set window #2 size to (%d,%d)\n", size.w, size.h);

  ret = nx_setsize(wstate.wndo[1].hwnd, &size);
  if (ret < 0)
    {
      printf("pwfb_main: nx_setsize failed: %d\n", errno);
      goto errout_with_hwnd2;
    }

  /* Set the position of window #2 */

  pt.x = wstate.xres / 4;
  pt.y = wstate.yres / 4;

  printf("pwfb_main: Set window #2 position to (%d,%d)\n", pt.x, pt.y);

  ret = nxtk_setposition(wstate.wndo[1].hwnd, &pt);
  if (ret < 0)
    {
      printf("pwfb_main: nxtk_setposition failed: %d\n", errno);
      goto errout_with_hwnd2;
    }

  /* Set up for motion */

  wstate.wndo[1].xmax   = itob32(wstate.xres - size.w - 1);
  wstate.wndo[1].ymax   = itob32(wstate.yres - size.h - 1);
  wstate.wndo[1].xpos   = itob32(pt.x);
  wstate.wndo[1].ypos   = itob32(pt.y);
  wstate.wndo[1].deltax = dtob32(-1.13)
  wstate.wndo[1].deltay = dtob32(5.0)

  /* Create window #3 */

  printf("pwfb_main: Create window #3\n");

  wstate.wndo[2].hwnd = nxtk_openwindow(wstate.hnx, NXBE_WINDOW_RAMBACKED,
                                        &g_nxcb, (FAR void *)&wstate);
  if (wstate.wndo[2].hwnd == NULL)
    {
      printf("pwfb_main: ERROR: nxtk_openwindow failed: %d\n", errno);
      goto errout_with_hwnd2;
    }

  printf("pwfb_main: hwnd3=%p\n", wstate.wndo[2].hwnd);

  /* Set the size of the window #3 == size of window 1*/

  printf("pwfb_main: Set window #3 size to (%d,%d)\n", size.w, size.h);

  ret = nx_setsize(wstate.wndo[2].hwnd, &size);
  if (ret < 0)
    {
      printf("pwfb_main: nx_setsize failed: %d\n", errno);
      goto errout_with_hwnd3;
    }

  /* Set the position of window #3 */

  pt.x = (3 * wstate.xres) / 8;
  pt.y = (3 * wstate.yres) / 8;

  printf("pwfb_main: Set window #3 position to (%d,%d)\n", pt.x, pt.y);

  ret = nxtk_setposition(wstate.wndo[2].hwnd, &pt);
  if (ret < 0)
    {
      printf("pwfb_main: nxtk_setposition failed: %d\n", errno);
      goto errout_with_hwnd3;
    }

  /* Set up for motion */

  wstate.wndo[2].xmax   = itob32(wstate.xres - size.w - 1);
  wstate.wndo[2].ymax   = itob32(wstate.yres - size.h - 1);
  wstate.wndo[2].xpos   = itob32(pt.x);
  wstate.wndo[2].ypos   = itob32(pt.y);
  wstate.wndo[2].deltax = dtob32(5.0)
  wstate.wndo[2].deltaxy= dtob32(-1.13)

  /* Now loop animating the windows */

  for (; ; )
    {
      nxeg_motion(&wstate);
    }

  /* Close window 3 */

errout_with_hwnd3:
  printf("pwfb_main: Close window #2\n");

  ret = nxtk_closewindow(wstate.wndo[2].hwnd);
  if (ret < 0)
    {
      printf("pwfb_main: ERROR: nxtk_closewindow failed: %d\n", errno);
      g_exitcode = NXEXIT_NXCLOSEWINDOW;
    }

  /* Close window 2 */

errout_with_hwnd2:
  printf("pwfb_main: Close window #2\n");

  int ret = nxtk_closewindow(wstate.wndo[1].hwnd);
  if (ret < 0)
    {
      printf("pwfb_main: ERROR: nxtk_closewindow failed: %d\n", errno);
      g_exitcode = NXEXIT_NXCLOSEWINDOW;
    }

  /* Close window1 */

errout_with_hwnd1:
  printf("pwfb_main: Close window #1\n");

  int ret = nxtk_closewindow(wstate.wndo[0].hwnd);
  if (ret < 0)
    {
      printf("pwfb_main: ERROR: nxtk_closewindow failed: %d\n", errno);
      g_exitcode = NXEXIT_NXCLOSEWINDOW;
    }

errout_with_nx:
  /* Disconnect from the server */

  printf("pwfb_main: Disconnect from the server\n");
  nx_disconnect(wstate.hnx);

errout:
  return g_exitcode;
}
