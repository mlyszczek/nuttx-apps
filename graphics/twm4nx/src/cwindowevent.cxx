// apps/graphics/twm4nx/src/cwindowevent.cxx
// Shim to manage the interface between NX messages and NxWidgets
//
//   Copyright (C) 2019 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <gnutt@nuttx.org>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name NuttX nor the names of its contributors may be
//    used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Included Files
/////////////////////////////////////////////////////////////////////////////

#include <nuttx/config.h>

#include <cfcntl>
#include <cerrno>

#include <mqueue.h>

#include "graphics/nxwidgets/cwidgetcontrol.hxx"

#include "graphics/twm4nx/twm4nx_config.hxx"
#include "graphics/twm4nx/cwindow.hxx"
#include "graphics/twm4nx/cwindowevent.hxx"

/////////////////////////////////////////////////////////////////////////////
// CWindowEvent Method Implementations
/////////////////////////////////////////////////////////////////////////////

using namespace Twm4Nx;

/**
 * CWindowEvent Constructor
 *
 * @param twm4nx The Twm4Nx session instance.
 * @param obj Contextual object (Usually 'this' of instantiator)
 * @param isBackground True is this for the background window.
 * @param style The default style that all widgets on this display
 *   should use.  If this is not specified, the widget will use the
 *   values stored in the defaultCWidgetStyle object.
 */

CWindowEvent::CWindowEvent(FAR CTwm4Nx *twm4nx, FAR void *obj,
                           bool isBackground,
                           FAR const NXWidgets::CWidgetStyle *style)
: NXWidgets::CWidgetControl(style)
{
  m_twm4nx       = twm4nx;       // Cache the Twm4Nx session
  m_object       = obj;          // Used for event message construction
  m_isBackground = isBackground; // Background window?

  // Open a message queue to send raw NX events.  This cannot fail!

  FAR const char *mqname = twm4nx->getEventQueueName();
  m_eventq = mq_open(mqname, O_WRONLY);
  if (m_eventq == (mqd_t)-1)
    {
      twmerr("ERROR: Failed open message queue '%s': %d\n",
             mqname, errno);
    }

  // Add ourself to the list of window event handlers

  addWindowEventHandler(this);
}

/**
 * CWindowEvent Destructor.
 */

CWindowEvent::~CWindowEvent(void)
{
 // Close the NxWidget event message queue

  if (m_eventq != (mqd_t)-1)
    {
      (void)mq_close(m_eventq);
      m_eventq = (mqd_t)-1;
    }

  // Remove ourself from the list of the window event handlers

  removeWindowEventHandler(this);
}

/**
 * Send the EVENT_MSG_POLL input event message to the Twm4Nx event loop.
 */

void CWindowEvent::sendInputEvent(void)
{
  twminfo("Input...\n");

  // The logic path here is tortuous but flexible:
  //
  //  1. A listener thread receives mouse or touchscreen input and injects
  //     that into NX via nx_mousein
  //  2. In the multi-user mode, this will send a message to the NX server
  //  3. The NX server will determine which window gets the mouse input
  //     and send a window event message to the NX listener thread.
  //  4. The NX listener thread receives a windows event.  The NX listener thread
  //     is part of CTwm4Nx and was created when NX server connection was
  //     established.  This event may be a positional change notification, a
  //     redraw request, or mouse or keyboard input.  In this case, mouse input.
  //  5. The NX listener thread handles the message by calling nx_eventhandler().
  //     nx_eventhandler() dispatches the message by calling a method in the
  //     NXWidgets::CCallback instance associated with the window.
  //     NXWidgets::CCallback is a part of the CWidgetControl.
  //  6. NXWidgets::CCallback calls into NXWidgets::CWidgetControl to process
  //     the event.
  //  7. NXWidgets::CWidgetControl records the new state data and raises a
  //     window event.
  //  8. NXWidgets::CWindowEventHandlerList will give the event to this method
  //     NxWM::CWindowEvent.
  //  9. This NxWM::CWindowEvent method will send a message to the Event
  //     loop running in the Twm4Nx main thread.
  // 10. The Twm4Nx main thread will call the CWindowEvent::event() method
  //     which
  // 11. Finally call pollEvents() to execute whatever actions the input event
  //     should trigger.
  // 12. This might call an event handler in and overrident method of
  //     CWidgetEventHandler which will again notify the Event loop running
  //     in the Twm4Nx main thread.  The event will, finally be delivered
  //     to the recipient in its fully digested and decorated form.

  struct SNxEventMsg msg;
  msg.eventID  = m_isBackground ? EVENT_BACKGROUND_POLL : EVENT_WINDOW_POLL;
  msg.instance = this;
  msg.obj      = m_object;

  int ret = mq_send(m_eventq, (FAR const char *)&msg,
                    sizeof(struct SNxEventMsg), 100);
  if (ret < 0)
    {
      twmerr("ERROR: mq_send failed: %d\n", ret);
    }
}

/**
 * Handle a NX window redraw request event
 *
 * @param nxRect The region in the window to be redrawn
 * @param more More redraw requests will follow
 */

void CWindowEvent::handleRedrawEvent(FAR const nxgl_rect_s *nxRect,
                                     bool more)
{
  twminfo("background=%s\n", m_isBackground ? "YES" : "NO");

  // At present, only the background window will get redraw events

  if (m_isBackground)
    {
      struct SRedrawEventMsg msg;
      msg.eventID = EVENT_BACKGROUND_REDRAW;
      msg.rect.pt1.x = nxRect->pt1.x;
      msg.rect.pt1.y = nxRect->pt1.y;
      msg.rect.pt2.x = nxRect->pt2.x;
      msg.rect.pt2.y = nxRect->pt2.y;
      msg.more       = more;

      // NOTE that we cannot block because we are on the same thread
      // as the message reader.  If the event queue becomes full then
      // we have no other option but to lose events.
      //
      // I suppose we could recurse and call Twm4Nx::dispatchEvent at
      // the risk of runaway stack usage.

      int ret = mq_send(m_eventq, (FAR const char *)&msg,
                        sizeof(struct SRedrawEventMsg), 100);
      if (ret < 0)
        {
          twmerr("ERROR: mq_send failed: %d\n", ret);
        }
    }
}

#ifdef CONFIG_NX_XYINPUT
/**
 * Handle an NX window mouse input event.
 */

void CWindowEvent::handleMouseEvent(void)
{
  twminfo("Mouse input...\n");

  // Stimulate an input poll

  sendInputEvent();
}
#endif

#ifdef CONFIG_NX_KBD
/**
 * Handle a NX window keyboard input event.
 */

void CWindowEvent::handleKeyboardEvent(void)
{
  twminfo("Keyboard input...\n");

  // Stimulate an input poll

  sendInputEvent();
}
#endif

/**
 * Handle a NX window blocked event.  This handler is called when we
 * receive the BLOCKED message meaning that there are no further pending
 * actions on the window.  It is now safe to delete the window.
 *
 * This is handled by sending a message to the start window thread (vs just
 * calling the destructors) because in the case where an application
 * destroys itself (because of pressing the stop button), then we need to
 * unwind and get out of the application logic before destroying all of its
 * objects.
 *
 * @param arg - User provided argument (see nx_block or nxtk_block)
 */

void CWindowEvent::handleBlockedEvent(FAR void *arg)
{
  twminfo("Blocked...\n");

  struct SNxEventMsg msg =
  {
    .eventID  = EVENT_WINDOW_DELETE,
    .instance = this,
    .obj      = arg
  };

  int ret = mq_send(m_eventq, (FAR const char *)&msg,
                    sizeof(struct SNxEventMsg), 100);
  if (ret < 0)
    {
      twmerr("ERROR: mq_send failed: %d\n", ret);
    }
}
