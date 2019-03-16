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

#include <errno.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxtk.h>

#include <pwfb_internal.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_setposition
 ****************************************************************************/

static inline int nxeg_setposition(NXEGWINDOW hwnd,
                                   FAR struct nxgl_point_s *pos)
{
  int ret = nxtk_setposition(hwnd, pos);
  if (ret < 0)
    {
      printf("nxeg_setposition: nxtk_setposition failed: %d\n", errno);
      g_exitcode = NXEXIT_NXSETPOSITION;
    }

  return ret;
}

/****************************************************************************
 * Name: nxeg_lower
 ****************************************************************************/

static inline int nxeg_lower(NXEGWINDOW hwnd)
{
  int ret = nxtk_lower(hwnd);
  if (ret < 0)
    {
      printf("nxeg_lower: nxtk_lower failed: %d\n", errno);
      g_exitcode = NXEXIT_NXLOWER;
    }

  return ret;
}

/****************************************************************************
 * Name: nxeg_raise
 ****************************************************************************/

static inline int nxeg_raise(NXEGWINDOW hwnd)
{
  int ret = nxtk_raise(hwnd);
  if (ret < 0)
    {
      printf("nxeg_raise: nxtk_raise failed: %d\n", errno);
      g_exitcode = NXEXIT_NXRAISE;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_motion
 ****************************************************************************/

void nxeg_motion(FAR struct nxeg_state_s *st)
{
}