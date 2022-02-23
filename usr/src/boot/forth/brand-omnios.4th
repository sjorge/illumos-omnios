\ Copyright (c) 2006-2015 Devin Teske <dteske@FreeBSD.org>
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

2 brandX ! 1 brandY ! \ Initialise brand placement defaults

: brand+ ( x y c-addr/u -- x y' )
	2swap 2dup at-xy 2swap	\ position the cursor
	[char] @ escc!		\ replace @ with Esc
	type			\ print to the screen
	1+			\ increase y for next time we're called
;

: asciitop ( x y -- x y' )
    s" @[m   ____   __  __  _   _  _         "                      brand+
    s"   / __ \ |  \/  || \ | || |        "                         brand+
    s"  | |  | ||      ||  \| || |        "                         brand+
    s"  | |__| || |\/| || , `@[33m_@[m||@[33m_@[m|  @[33m____@[m  " brand+
    s"   \____/ |_|  |_||_|\@[33m/ __ \ / ___| "                    brand+
    s"                     | |  | ||(__   "                         brand+
    s"         @[30;1mcommunity@[0;33m   | |__| | ___)| "           brand+
    s"              @[30;1medition@[0;33m \____/ |____/@[m "        brand+
;

: graphtop ( x y -- x y )
	pngdebug @
	s" term-putimage" sfind if
		>r
		1 0 30 0	\ top left at (1,0), bottom right at (30,0)
				\ the 0 preserves aspect ratio.
	else
		['] fb-putimage >r
		30 20 0 0
	then
	s" /boot/ooce.png"
	r> execute
	invert if asciitop then	\ fall-back to ASCII version
;

: ooceversion ( -- )
	s" ooce_version" getenv dup -1 = if
		drop				\ ooce_version not set
	else
		dup sc swap - 2/ 1 at-xy	\ Centre on row 1
		2 fg b				\ Green bold
		type				\ Output
		me				\ Mode end
	then
;

: brand ( x y -- )
	framebuffer? if graphtop else asciitop then
	ooceversion
	2drop
;

