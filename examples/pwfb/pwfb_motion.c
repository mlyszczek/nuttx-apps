/****************************************************************************
 * examples/pwfb/pwfb_motion.c
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

#include <stdbool.h>
#include <errno.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxtk.h>

#include <pwfb_internal.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_move_window
 ****************************************************************************/

static inline void nxeg_move_window(FAR struct nxeg_state_s *st, int i)
{
  FAR struct nxeg_window_s *wndo = &st->wndo[i];
  FAR struct nxgl_point_s pos;
  b32_t newx;
  b32_t newy;
  bool hit = false;
  int ret;

  /* Update X position */

  newx             = wndo->xpos + wndo->deltax;

  /* Check for collision with left or right side */

  if (newx <= 0)
    {
      newx         = 0;
      wndo->deltax = -wndo->deltax;
      hit          = true;
    }
  else if (newx >= wndo->xmax)
    {
      newx         = wndo->xmax;
      wndo->deltax = -wndo->deltax;
      hit          = true;
    }

  wndo->xpos       = newx;

  /* Update Y position */

  newy             = wndo->ypos + wndo->deltay;

  /* Check for collision with top or bottom side */

  if (newy <= 0)
    {
      newy         = 0;
      wndo->deltay = -wndo->deltay;
      hit          = true;
    }
  else if (newy >= wndo->ymax)
    {
      newy         = wndo->ymax;
      wndo->deltay = -wndo->deltay;
      hit          = true;
    }

  wndo->ypos       = newy;

  /* Set the new window position */

  pos.x            = b32toi(newx);
  pos.y            = b32toi(newy);
  ret              = nxtk_setposition(st->hwnd, &pos);
  if (ret < 0)
    {
      printf("nxeg_move_window: nxtk_setposition failed: %d\n", errno);
    }

  /* If we hit an edge, the raise the window */

  ret              = nxtk_raise(st->hwnd);
  if (ret < 0)
    {
      printf("nxeg_move_window: nxtk_raise failed: %d\n", errno);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_motion
 ****************************************************************************/

void nxeg_motion(FAR struct nxeg_state_s *st)
{
  int i;

  /* Move each window */

  for (i = 0; i < 3; i++)
    {
      nxeg_move_window(st, i);
    }
}