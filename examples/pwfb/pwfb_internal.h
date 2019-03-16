/****************************************************************************
 * examples/pwfb/pwfb_internal.h
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

#ifndef __EXAMPLES_PWFB_PWFB_INTERNAL_H
#define __EXAMPLES_PWFB_PWFB_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <fixedmath.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxfonts.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NX
#  error "NX is not enabled (CONFIG_NX)"
#endif

#ifndef CONFIG_EXAMPLES_PWFB_VPLANE
#    define CONFIG_EXAMPLES_PWFB_VPLANE 0
#endif

#ifndef CONFIG_EXAMPLES_PWFB_BPP
#  define CONFIG_EXAMPLES_PWFB_BPP 32
#endif

#ifndef CONFIG_EXAMPLES_PWFB_BGCOLOR
#  if CONFIG_EXAMPLES_PWFB_BPP == 24 || CONFIG_EXAMPLES_PWFB_BPP == 32
#    define CONFIG_EXAMPLES_PWFB_BGCOLOR 0x007b68ee
#  elif CONFIG_EXAMPLES_PWFB_BPP == 16
#    define CONFIG_EXAMPLES_PWFB_BGCOLOR 0x7b5d
#  else
#    define CONFIG_EXAMPLES_PWFB_BGCOLOR ' '
# endif
#endif

#ifndef CONFIG_EXAMPLES_PWFB_COLOR1
#  if CONFIG_EXAMPLES_PWFB_BPP == 24 || CONFIG_EXAMPLES_PWFB_BPP == 32
#    define CONFIG_EXAMPLES_PWFB_COLOR1 0x00e6e6fa
#  elif CONFIG_EXAMPLES_PWFB_BPP == 16
#    define CONFIG_EXAMPLES_PWFB_COLOR1 0xe73f
#  else
#    define CONFIG_EXAMPLES_PWFB_COLOR1 '1'
# endif
#endif

#ifndef CONFIG_EXAMPLES_PWFB_COLOR2
#  if CONFIG_EXAMPLES_PWFB_BPP == 24 || CONFIG_EXAMPLES_PWFB_BPP == 32
#    define CONFIG_EXAMPLES_PWFB_COLOR2 0x00dcdcdc
#  elif CONFIG_EXAMPLES_PWFB_BPP == 16
#    define CONFIG_EXAMPLES_PWFB_COLOR2 0xdefb
#  else
#    define CONFIG_EXAMPLES_PWFB_COLOR2 '2'
# endif
#endif

#ifndef CONFIG_EXAMPLES_PWFB_TBCOLOR
#  if CONFIG_EXAMPLES_PWFB_BPP == 24 || CONFIG_EXAMPLES_PWFB_BPP == 32
#    define CONFIG_EXAMPLES_PWFB_TBCOLOR 0x00a9a9a9
#  elif CONFIG_EXAMPLES_PWFB_BPP == 16
#    define CONFIG_EXAMPLES_PWFB_TBCOLOR 0xad55
#  else
#    define CONFIG_EXAMPLES_PWFB_TBCOLOR 'T'
#  endif
#endif

#ifndef CONFIG_EXAMPLES_PWFB_FONTID
#  define CONFIG_EXAMPLES_PWFB_FONTID NXFONT_DEFAULT
#endif

#ifndef CONFIG_EXAMPLES_PWFB_FONTCOLOR
#  if CONFIG_EXAMPLES_PWFB_BPP == 24 || CONFIG_EXAMPLES_PWFB_BPP == 32
#    define CONFIG_EXAMPLES_PWFB_FONTCOLOR 0x00000000
#  elif CONFIG_EXAMPLES_PWFB_BPP == 16
#    define CONFIG_EXAMPLES_PWFB_FONTCOLOR 0x0000
#  else
#    define CONFIG_EXAMPLES_PWFB_FONTCOLOR 'F'
#  endif
#endif

#ifndef CONFIG_EXAMPLES_PWFB_TOOLBAR_HEIGHT
#  define CONFIG_EXAMPLES_PWFB_TOOLBAR_HEIGHT 16
#endif

/* NX Server Options */

#ifdef CONFIG_DISABLE_MQUEUE
#  error "The multi-threaded example requires MQ support (CONFIG_DISABLE_MQUEUE=n)"
#endif
#ifdef CONFIG_DISABLE_SIGNALS
#  error "This example requires signal support (CONFIG_DISABLE_SIGNALS=n)"
#endif
#ifdef CONFIG_DISABLE_PTHREAD
#  error "This example requires pthread support (CONFIG_DISABLE_PTHREAD=n)"
#endif
#ifndef CONFIG_NX_BLOCKING
#  error "This example depends on CONFIG_NX_BLOCKING"
#endif
#ifndef CONFIG_EXAMPLES_PWFB_STACKSIZE
#  define CONFIG_EXAMPLES_PWFB_STACKSIZE 2048
#endif
#ifndef CONFIG_EXAMPLES_PWFB_LISTENERPRIO
#  define CONFIG_EXAMPLES_PWFB_LISTENERPRIO 100
#endif
#ifndef CONFIG_EXAMPLES_PWFB_CLIENTPRIO
#  define CONFIG_EXAMPLES_PWFB_CLIENTPRIO 100
#endif
#ifndef CONFIG_EXAMPLES_PWFB_SERVERPRIO
#  define CONFIG_EXAMPLES_PWFB_SERVERPRIO 120
#endif
#ifndef CONFIG_EXAMPLES_PWFB_NOTIFYSIGNO
#  define CONFIG_EXAMPLES_PWFB_NOTIFYSIGNO 4
#endif

#define NXTK_MAXKBDCHARS 16

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum exitcode_e
{
  NXEXIT_SUCCESS = 0,
  NXEXIT_SIGPROCMASK,
  NXEXIT_SCHEDSETPARAM,
  NXEXIT_EVENTNOTIFY,
  NXEXIT_TASKCREATE,
  NXEXIT_PTHREADCREATE,
  NXEXIT_EXTINITIALIZE,
  NXEXIT_FBINITIALIZE,
  NXEXIT_FBGETVPLANE,
  NXEXIT_LCDINITIALIZE,
  NXEXIT_LCDGETDEV,
  NXEXIT_NXOPEN,
  NXEXIT_FONTOPEN,
  NXEXIT_NXOPENTOOLBAR,
  NXEXIT_NXCONNECT,
  NXEXIT_NXSETBGCOLOR,
  NXEXIT_NXOPENWINDOW,
  NXEXIT_NXSETSIZE,
  NXEXIT_NXSETPOSITION,
  NXEXIT_NXLOWER,
  NXEXIT_NXRAISE,
  NXEXIT_NXCLOSEWINDOW,
  NXEXIT_LOSTSERVERCONN
};

/* Describes one cached glyph bitmap */

struct nxeg_glyph_s
{
  uint8_t code;                        /* Character code */
  uint8_t height;                      /* Height of this glyph (in rows) */
  uint8_t width;                       /* Width of this glyph (in pixels) */
  uint8_t stride;                      /* Width of the glyph row (in bytes) */
  FAR uint8_t *bitmap;                 /* Allocated bitmap memory */
};

/* Describes on character on the display */

struct nxeg_bitmap_s
{
  struct nxgl_rect_s bounds;               /* Size/position of bitmap */
  FAR const struct nxeg_glyph_s *glyph;    /* The cached glyph */
};

/* Describes the unique state of one window */

struct nxeg_window_s
{
  NXEGWINDOW hwnd;                         /* Window handle */
  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]; /* Window color */
  b32_t xmax;                              /* Max X position */
  b32_t ymax;                              /* Max Y position */
  b32_t xpos;                              /* Current X position */
  b32_t ypos;                              /* Current Y position */
  b32_t deltax;                            /* Current X speed */
  b32_t deltay;                            /* Current Y speed */
};

/* Describes the overall state of the example */

struct nxeg_state_s
{
  /* NX server */

  volatile bool haveres;                   /* True:  Have screen resolution */
  volatile bool connected;                 /* True:  Connected to server */
  sem_t semevent;                          /* Event wait semaphore */
  NXHANDLE hnx;                            /* Connection handle */

  /* Font cache */

  FCACHE fcache;                           /* Font cache handle */
  NXHANDLE hfont;                          /* The font handle */

  /* Font properties */

  uint8_t fheight;                         /* Max height of a font in pixels */
  uint8_t fwidth;                          /* Max width of a font in pixels */
  uint8_t spwidth;                         /* The width of a space */

  /* Window-specific state */

  struct nxeg_window_s wndo[3];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* NX callback vtables */

extern const struct nx_callback_s g_nxcb;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR void *nx_listenerthread(FAR void *arg);
void nxeg_motion(FAR struct nxeg_state_s *st);

#endif /* __EXAMPLES_PWFB_PWFB_INTERNAL_H */
