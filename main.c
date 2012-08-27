/*
 * main.c - BTHeater 1.0
 * Copyright (C) 2012 Mansour <mansour@oxplot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define F_CPU 8000000
#define BAUD 38400
#define MYUBRR (F_CPU / 16 / BAUD - 1)

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

#define INITIAL_WAIT 2000
#define INQ_STR "\r\n+INQ=1\r\n"
#define MAX_LINE_LEN 15
#define EEPROM_DEFAULT 0
#define EEPROM_DEFAULT_L 0
#define EEPROM_DEFAULT_R 1

#define PORT_L1 PORTD2
#define PORT_L2 PORTD5
#define PORT_R1 PORTD3
#define PORT_R2 PORTD6
#define PIN_L1 PIND2
#define PIN_L2 PIND5
#define PIN_R1 PIND3
#define PIN_R2 PIND6

#define CMD_VIEW 'v'
#define CMD_HELP '?'
#define CMD_ON '1'
#define CMD_OFF '0'
#define CMD_TOGGLE 'g'
#define CMD_TIMEOUT 't'
#define CMD_INTERVAL 'i'
#define CMD_DEFAULT 'd'
#define CMD_CLEAR 'c'
#define CMD_LIST 'l'
#define SOCK_LEFT 'l'
#define SOCK_RIGHT 'r'
#define SOCK_ALL 'a'
#define REPLY_OK "OK\r\n"
#define REPLY_ERROR "ERROR\r\n"

#define IS_SOCK_VALID(s) (s == SOCK_LEFT || s == SOCK_RIGHT \
                          || s == SOCK_ALL)
#define IS_SCMD_VALID(s) (s == CMD_ON || s == CMD_OFF \
                          || s == CMD_TOGGLE)
#define IS_SCMD2_VALID(s) (s == CMD_ON || s == CMD_OFF)

#define SLOT_COUNT 9
#define SLOT_FLAG_SET 1
#define SLOT_FLAG_L 2
#define SLOT_FLAG_R 4
#define SLOT_FLAG_INT 8
#define SLOT_FLAG_ON 16
#define SLOT_FLAG_TOGGLE 32

#define HELP_CONTENT2 "COMMANDS\r\n" \
"  V   Show state of the sockets.\r\n" \
"  0   Turn off a socket.\r\n" \
"  1   Turn on a socket.\r\n" \
"  G   Toggle a socket.\r\n" \
"  D   Set default state of a socket on startup.\r\n" \
"  T   Set a timeout.\r\n" \
"  I   Set an interval.\r\n" \
"  C   Clear a timeout/interval.\r\n" \
"  L   List all timeouts/intervals.\r\n" \
"  ?   Print this help.\r\n"

struct slot {
  unsigned char flags;
  /* Timeout timestamp */
  unsigned int ts;
  /* Interval */
  unsigned int iv;
} slots[SLOT_COUNT];

/* Current timestamp */
unsigned int cts;

void main(void) __attribute__ ((noreturn));
static void init(unsigned int);
static inline void USART_send(unsigned char const);
static inline void USART_send_u(unsigned char const);
static inline unsigned char USART_recv(void);
static void USART_send_str(char const *);
static void USART_send_str_u(char const *);
static inline unsigned short strlen(char const *);
static void process_interval(void);
static unsigned short slice(char const *, unsigned short const, char const,
  char *, unsigned short const);
static unsigned char parse_timestamp(char const *, unsigned int *);
static unsigned int atoi(char const *);
static void print_list(void);
static void print_timestamp(char *, unsigned int const);
static unsigned short read_line(char *, unsigned short const);
static void process_timeout(void);
static void process_default(void);
static void process_clear(void);
static void process_action(char const);

void
main(void) {

  sei();

  /* Set things up */

  init(MYUBRR);

  /* Let everybody warm up */

  _delay_ms(INITIAL_WAIT);

  /* Put Bluetooth into inquiry mode */

  USART_send_str_u(INQ_STR);

  /* Endless loop */

  char buf[MAX_LINE_LEN + 1];
  unsigned short blen = 0;

  while (1) {
    blen = read_line(buf, MAX_LINE_LEN);
    if (blen != 1)
      continue;

    char cmd = buf[0];

    if (cmd == CMD_HELP) {
      USART_send_str(HELP_CONTENT2);

    } else if (cmd == CMD_VIEW) {
      unsigned char pind, def;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pind = PIND;
        def = eeprom_read_byte(EEPROM_DEFAULT);
      }
      USART_send_str(" Left is ");
      USART_send_str(pind & (1 << PIN_L1) ? "ON" : "OFF");
      USART_send_str(" (startup: ");
      USART_send_str(def & (1 << EEPROM_DEFAULT_L) ? "on" : "off");
      USART_send_str(")\r\n");
      USART_send_str(" Right is ");
      USART_send_str(pind & (1 << PIN_R1) ? "ON" : "OFF");
      USART_send_str(" (startup: ");
      USART_send_str(def & (1 << EEPROM_DEFAULT_R) ? "on" : "off");
      USART_send_str(")\r\n");
      USART_send_str("\r\n");

    } else if (cmd == CMD_DEFAULT) {
      process_default();

    } else if (cmd == CMD_TIMEOUT) {
      process_timeout();

    } else if (cmd == CMD_INTERVAL) {
      process_interval();

    } else if (cmd == CMD_CLEAR) {
      process_clear();

    } else if (cmd == CMD_LIST) {
      print_list();

    } else if (cmd == CMD_ON || cmd == CMD_OFF || cmd == CMD_TOGGLE) {
      process_action(cmd);
    }
  }
}

static unsigned short
read_line(char *buf, unsigned short const max_len)
{
  static char lastc;
  unsigned short i = 0;
  char c = USART_recv();
  if (lastc == '\r' && c == '\n')
    c = USART_recv();
  while (i < max_len && c != '\r' && c != '\n') {
    if (c >= 'A' && c <= 'Z')
      c = c - 'A' + 'a';
    buf[i++] = c;
    c = USART_recv();
  }
  lastc = c;
  buf[i] = '\0';
  return i;
}

static void
init(unsigned int const ubrr)
{

  /* Read the default states from EEPROM */
  PORTD = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    unsigned char s = eeprom_read_byte(EEPROM_DEFAULT);
    if (s & (1 << EEPROM_DEFAULT_L))
      PORTD |= (1 << PORT_L1) | (1 << PORT_L2);
    if (s & (1 << EEPROM_DEFAULT_R))
      PORTD |= (1 << PORT_R1) | (1 << PORT_R2);
  }
  
  /* Relay drivers */

  DDRD |= (1 << DDD2) | (1 << DDD3) | (1 << DDD5) | (1 << DDD6);

  /* Setup USART */

  UBRR0H = (unsigned char) (ubrr >> 8);
  UBRR0L = (unsigned char) ubrr;
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  /* Set up a timer that fires every second */

  OCR1A = 31250;
  TCCR1B |= (1 << WGM12) | (1 << CS12);
  TIMSK1 |= 1 << OCIE1A;

  /* Initialize the slots */

  unsigned char i;
  for (i = 0; i < SLOT_COUNT; i++)
    slots[i].flags = 0;
  cts = 0;
}

ISR(TIMER1_COMPA_vect)
{
  cts++;
  unsigned char i;
  for (i = 0; i < SLOT_COUNT; i++) {
    struct slot *s = slots + i;
    if (!(s->flags & SLOT_FLAG_SET))
      continue;
    if (s->flags & SLOT_FLAG_INT) {
      if (s->ts > cts
          || (s->ts < cts && ((cts - s->ts) % s->iv) != 0))
        continue;
    } else {
      if (s->ts < cts)
        s->flags = 0;
      if (s->ts != cts)
        continue;
    }

    unsigned char portd = PORTD;
    unsigned char org_portd = portd;
    unsigned char ds = 0;

    if (s->flags & SLOT_FLAG_L) {
      portd &= ~((1 << PORT_L1) | (1 << PORT_L2));
      ds |= (1 << PORT_L1) | (1 << PORT_L2);
    }
    if (s->flags & SLOT_FLAG_R) {
      portd &= ~((1 << PORT_R1) | (1 << PORT_R2));
      ds |= (1 << PORT_R1) | (1 << PORT_R2);
    }

    if (s->flags & SLOT_FLAG_TOGGLE)
      PORTD = org_portd ^ ds;
    else if (s->flags & SLOT_FLAG_ON)
      PORTD = portd | ds;
    else
      PORTD = portd;
    
  }
}

static inline void
USART_send(unsigned char const data)
{
  while (!(UCSR0A & (1 << UDRE0))) {}
  if (PIND & (1 << PIND4))
    UDR0 = data;
}

static inline void
USART_send_u(unsigned char const data)
{
  while (!(UCSR0A & (1 << UDRE0))) {}
  UDR0 = data;
}

static inline unsigned char
USART_recv(void)
{
  while (1) {
    while (!(UCSR0A & (1 << RXC0))) {}
    if (PIND & (1 << PIND4))
      return UDR0;
    else {
      UDR0;
    }
  }
}

static void
USART_send_str(char const *str)
{
  while (*str != '\0')
    USART_send(*str++);
}

static void
USART_send_str_u(char const *str)
{
  while (*str != '\0')
    USART_send_u(*str++);
}

static inline unsigned short
strlen(char const *str)
{
  unsigned short i = 0;
  while (*(str++) != '\0')
    i++;
  return i;
}

static unsigned short
slice(char const *str, unsigned short const start, char const stop,
      char *buf, unsigned short const max_len)
{
  str += start;
  unsigned short l = 0;
  while (*str != '\0' && *str != stop && l < max_len)
    buf[l++] = *str++;
  buf[l] = '\0';
  return start + l + (*str == stop ? 1 : 0);
}

static void
print_timestamp(char *buf, unsigned int const ts)
{
  unsigned char hr = ts / 3600, min = (ts % 3600) / 60, sec = ts % 60;
  buf[0] = hr / 10 + '0';
  buf[1] = hr % 10 + '0';
  buf[2] = ':';
  buf[3] = min / 10 + '0';
  buf[4] = min % 10 + '0';
  buf[5] = ':';
  buf[6] = sec / 10 + '0';
  buf[7] = sec % 10 + '0';
  buf[8] = '\0';
}

static void
print_list(void)
{
  unsigned char i;
  char buf[MAX_LINE_LEN + 1];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (i = 0; i < SLOT_COUNT; i++) {
      struct slot *s = slots + i;
      USART_send(' ');
      USART_send('1' + i);
      USART_send(' ');
      if (!(s->flags & SLOT_FLAG_SET)) {
        USART_send_str("-\r\n");
        continue;
      }
      USART_send_str(s->flags & SLOT_FLAG_TOGGLE ? "toggle " :
        (s->flags & SLOT_FLAG_ON ? "turn on " : "turn off "));
      USART_send_str(s->flags & SLOT_FLAG_L
        && s->flags & SLOT_FLAG_R ? "All " :
        (s->flags & SLOT_FLAG_L ? "Left " : "Right "));
      if (s->flags & SLOT_FLAG_INT) {
        USART_send_str("every ");
        print_timestamp(buf, s->iv);
        USART_send_str(buf);
        USART_send_str(", next in ");
        print_timestamp(buf, s->ts > cts ?
                        s->ts - cts : ((s->ts - cts) % s->iv));
        USART_send_str(buf);
      } else {
        USART_send_str("in ");
        print_timestamp(buf, s->ts - cts);
        USART_send_str(buf);
      }
      USART_send_str("\r\n");
    }
  }
  USART_send_str("\r\n");
}

static unsigned int
atoi(char const *str)
{
  unsigned int o = 0;
  while (*str != '\0') {
    if (*str >= '0' && *str <= '9')
      o = o * 10 + (*str - '0');
    str++;
  }
  return o;
}

static unsigned char
parse_timestamp(char const *buf, unsigned int *ts)
{
  char dbuf[3][3];
  unsigned int start = 0;
  int i;
  for (i = 2; i >= 0; i--)
    start = slice(buf, start, ':', dbuf[i], 2);
  i = 0;
  while (strlen(dbuf[i]) == 0)
    i++;

  /* Read the second at the least */

  if (i > 2)
    return 0;
  *ts = atoi(dbuf[i++]);
  if (i > 2)
    return 1;
  *ts += atoi(dbuf[i++]) * 60;
  if (i > 2)
    return 1;
  *ts += atoi(dbuf[i++]) * 3600;
  return 1;
}

static void
process_timeout(void)
{
  char buf[MAX_LINE_LEN + 1];
  unsigned short blen;

  /* Read the socket */

  do {
    USART_send_str("Enter socket, (L)eft, (R)ight or (A)ll:\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
  } while (blen != 1 || !IS_SOCK_VALID(buf[0]));
  char sock = buf[0];

  /* Read the state command */

  do {
    USART_send_str(
      "Enter action, (0) Turn off, (1) Turn on, (G) Toggle:\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
  } while (blen != 1 || !IS_SCMD_VALID(buf[0]));
  char scmd = buf[0];

  /* Read the start time */

  unsigned int sspan;
  do {
    USART_send_str("Enter start as [[hh:]mm:]ss:\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
  } while (blen < 1 || blen > 8 || !parse_timestamp(buf, &sspan));

  /* Set an empty slot to one of the available slots */

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    unsigned char i;
    for (i = 0; i < SLOT_COUNT; i++) {
      struct slot *s = slots + i;
      if (!(s->flags & SLOT_FLAG_SET)) {
        s->flags = SLOT_FLAG_SET
          | (scmd == CMD_TOGGLE ? SLOT_FLAG_TOGGLE :
            (scmd == CMD_ON ? SLOT_FLAG_ON : 0))
          | (sock == SOCK_ALL ? (SLOT_FLAG_L | SLOT_FLAG_R) :
            (sock == SOCK_LEFT ? SLOT_FLAG_L : SLOT_FLAG_R));
        s->ts = cts + sspan;
        USART_send_str("OK ");
        USART_send('1' + i);
        USART_send_str("\r\n");
        return;
      }
    }
  }

  /* No available slot */

  USART_send_str("FULL, see 'C' cmd.\r\n");
}

static void
process_interval(void)
{
  char buf[MAX_LINE_LEN + 1];
  unsigned short blen;

  /* Read the socket */

  do {
    USART_send_str("Enter socket, (L)eft, (R)ight or (A)ll:\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
  } while (blen != 1 || !IS_SOCK_VALID(buf[0]));
  char sock = buf[0];

  /* Read the state command */

  do {
    USART_send_str(
      "Enter action, (0) Turn off, (1) Turn on, (G) Toggle:\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
  } while (blen != 1 || !IS_SCMD_VALID(buf[0]));
  char scmd = buf[0];

  /* Read the interval */

  unsigned int ispan;
  do {
    USART_send_str("Enter interval as [[hh:]mm:]ss:\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
  } while (blen < 1 || blen > 8 || !parse_timestamp(buf, &ispan));

  /* Read the start time */

  unsigned int sspan;
  unsigned char sset = 1;
  do {
    USART_send_str("Enter start as [[hh:]mm:]ss (optional):\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
    if (blen == 0) {
      sset = 0;
      break;
    }
  } while (blen > 8 || !parse_timestamp(buf, &sspan));

  /* Set an empty slot to one of the available slots */

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    unsigned char i;
    for (i = 0; i < SLOT_COUNT; i++) {
      struct slot *s = slots + i;
      if (!(s->flags & SLOT_FLAG_SET)) {
        s->flags = SLOT_FLAG_SET | SLOT_FLAG_INT
          | (scmd == CMD_TOGGLE ? SLOT_FLAG_TOGGLE :
            (scmd == CMD_ON ? SLOT_FLAG_ON : 0))
          | (sock == SOCK_ALL ? (SLOT_FLAG_L | SLOT_FLAG_R) :
            (sock == SOCK_LEFT ? SLOT_FLAG_L : SLOT_FLAG_R));
        s->ts = cts + (sset ? sspan : ispan);
        s->iv = ispan;
        USART_send_str("OK ");
        USART_send('1' + i);
        USART_send_str("\r\n");
        return;
      }
    }
  }

  /* No available slot */

  USART_send_str("ALL SLOTS FULL. Use 'C' command to free one up.\r\n");
}

static void
process_default(void)
{
  char buf[MAX_LINE_LEN + 1];
  unsigned short blen;

  /* Read the socket */

  do {
    USART_send_str("Enter socket, (L)eft, (R)ight or (A)ll:\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
  } while (blen != 1 || !IS_SOCK_VALID(buf[0]));
  char sock = buf[0];

  /* Read the state command */

  do {
    USART_send_str(
      "Enter default state, (0) Off, (1) On or (C)urrent:\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
  } while (blen != 1 || (!IS_SCMD2_VALID(buf[0]) && buf[0] != 'c'));
  char scmd = buf[0];

  /* Write the default setup to EEPROM */

  unsigned char ds;
  if (scmd == 'C') {
    unsigned char pind;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      pind = PIND;
    }
    ds = ((pind & (1 << PIN_L1) >> PIN_L1) << EEPROM_DEFAULT_L)
      | ((pind & (1 << PIN_R1) >> PIN_R1) << EEPROM_DEFAULT_R);
  } else if (scmd == '1') {
    ds = (1 << EEPROM_DEFAULT_L) | (1 << EEPROM_DEFAULT_R);
  } else {
    ds = 0;
  }

  unsigned char old, new;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    old = new = eeprom_read_byte(EEPROM_DEFAULT);
  }
  if (sock == SOCK_ALL) {
    new = ds;
  } else if (sock == SOCK_LEFT) {
    new = (new & ~(1 << EEPROM_DEFAULT_L))
      | (ds & (1 << EEPROM_DEFAULT_L));
  } else {
    new = (new & ~(1 << EEPROM_DEFAULT_R))
      | (ds & (1 << EEPROM_DEFAULT_R));
  }

  if (old != new) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      eeprom_write_byte(EEPROM_DEFAULT, new);
    }
  }

  USART_send_str(REPLY_OK);
}

static void
process_clear(void)
{
  char buf[MAX_LINE_LEN + 1];
  unsigned short blen;

  USART_send_str("TIMEOUTS AND INTERVALS\r\n\r\n");
  print_list();
  do {
    USART_send_str(
      "Enter slot num 1-9 or (A)ll:\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
  } while (blen != 1
    || ((buf[0] < '0' || buf[0] > '9') && buf[0] != 'a'));
  unsigned char slotn = buf[0];
  if (slotn == SOCK_ALL) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      unsigned char i;
      for (i = 0; i < SLOT_COUNT; i++)
        slots[i].flags = 0;
    }
  } else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      slots[slotn - '1'].flags = 0;
    }
  }
  USART_send_str(REPLY_OK);
}

static void
process_action(char const cmd)
{
  char buf[MAX_LINE_LEN + 1];
  unsigned short blen;

  /* Read the socket */

  do {
    USART_send_str("Enter socket, (L)eft, (R)ight or (A)ll:\r\n");
    blen = read_line(buf, MAX_LINE_LEN);
  } while (blen != 1 || !IS_SOCK_VALID(buf[0]));
  char sock = buf[0];

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    unsigned char portd = PORTD;
    unsigned char org_portd = portd;
    unsigned char ds = 0;

    if (sock == SOCK_ALL || sock == SOCK_LEFT) {
      portd &= ~((1 << PORT_L1) | (1 << PORT_L2));
      ds |= (1 << PORT_L1) | (1 << PORT_L2);
    }
    if (sock == SOCK_ALL || sock == SOCK_RIGHT) {
      portd &= ~((1 << PORT_R1) | (1 << PORT_R2));
      ds |= (1 << PORT_R1) | (1 << PORT_R2);
    }

    if (cmd == CMD_ON)
      PORTD = portd | ds;
    else if (cmd == CMD_TOGGLE)
      PORTD = org_portd ^ ds;
    else
      PORTD = portd;
  }
  USART_send_str(REPLY_OK);
}
