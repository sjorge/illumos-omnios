\ Copyright (c) 2003 Scott Long <scottl@FreeBSD.org>
\ Copyright (c) 2003 Aleksander Fafula <alex@fafula.com>
\ Copyright (c) 2006-2015 Devin Teske <dteske@FreeBSD.org>
\ Copyright 2017 Dominik Hassler <hadfl@cpan.org>
\ Copyright 2020 OmniOS Community Edition (OmniOSce) Association.
\ All rights reserved.
\
\ Redistribution and use in source and binary forms, with or without
\ modification, are permitted provided that the following conditions
\ are met:
\ 1. Redistributions of source code must retain the above copyright
\    notice, this list of conditions and the following disclaimer.
\ 2. Redistributions in binary form must reproduce the above copyright
\    notice, this list of conditions and the following disclaimer in the
\    documentation and/or other materials provided with the distribution.
\
\ THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
\ ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
\ IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
\ ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
\ FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
\ DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
\ OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
\ HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
\ LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
\ OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
\ SUCH DAMAGE.

51 logoX !
2 logoY !

variable pngdebug
0 pngdebug !

: logo+ ( x y c-addr/u -- x y' )
	2swap 2dup at-xy 2swap	\ position the cursor
	[char] @ escc!		\ replace @ with Esc
	type			\ print to the screen
	1+			\ increase y for next time we're called
;

: illumos_logo ( -- )
	pngdebug @
	0 0 0 0		\ bottom right, no scaling
	s" /boot/illumos-small.png"
	fb-putimage drop
;

: asciifenix ( x y -- x y' )
	s"     @[30;1m.:   ..                 " logo+
	s"    .o,  .o.                 "        logo+
	s"    :d.  ld.   .'            "        logo+
	s"    cd;  cd;   l.            "        logo+
	s"    .dd'  ld,  cc            "        logo+
	s"     .;oc. 'l:. c;           "        logo+
	s"        .''...;;.,c.         "        logo+
	s"               'c:.'         "        logo+
	s"  .c.                        "        logo+
	s" .ddd::loo;.    ;:.          "        logo+
	s" ;ddddddl.      .;;:.        "        logo+
	s" :dddddl           lddo,     "        logo+
	s" .ddddd;             .;do    "        logo+
	s"  .odddo.              .o    "        logo+
	s"    ;dddo,              .    "        logo+
	s"      ,lddo,.;'              "        logo+
	s"        .;oddd:.             "        logo+
	s"         ...;oddl,           "        logo+
	s"          ,l,.'cdddc.        "        logo+
	s"         .cdddc..:dddc.      "        logo+
	s"            .;odc..,oddl.    "        logo+
	s"             .odddl'.;dddc.  "        logo+
	s"             :ddddddl..cddo'@[m "     logo+
;

: graphfenix ( x y -- x y )
	s" term-putimage" sfind if
		>r
		2dup pngdebug @ -rot	( x y -- x y d x y )
		0 26			\ end on row 26, keep aspect ratio
	else
		['] fb-putimage >r
		pngdebug @
		530 30 0 0
	then
	s" /boot/fenix.png"
	r> execute
	\ Fall-back to the ASCII version
	invert if asciifenix then
;

: logo ( x y -- )
	framebuffer? if
		s" loader_font" set_font
		clear at-bl
		50 1 graphfenix 2drop
		illumos_logo
	else
		asciifenix
	then

	at-bl
	2drop
;

