/*
 * pi_link.h - binary link to a LinuxCNC host over the v9 board's J17 header (USART2)
 * This file is part of the g2core project
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * Protocol: see PROTOCOL.md in this directory. v0 is echo and status only.
 * Built only with `make ... PI_LINK=1`; without it this code is not compiled at all.
 */

#ifndef PI_LINK_H_ONCE
#define PI_LINK_H_ONCE

#include "g2core.h"

#ifndef PI_LINK_BAUD
#define PI_LINK_BAUD 115200
#endif

void pi_link_init();        // call once from hardware_init()
stat_t pi_link_periodic();  // call from hardware_periodic(); never blocks

#endif  // End of include guard: PI_LINK_H_ONCE
