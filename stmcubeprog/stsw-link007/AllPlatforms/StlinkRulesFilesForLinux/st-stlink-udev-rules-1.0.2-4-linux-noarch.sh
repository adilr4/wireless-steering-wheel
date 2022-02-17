#!/bin/sh
# This script was generated using Makeself 2.2.0

umask 077

CRCsum="1532774049"
MD5="a6562597433f4369c899d2f345377f2a"
TMPROOT=${TMPDIR:=/tmp}

label="STM udev rules installer"
script="./setup.sh"
scriptargs="1.0.2-4"
licensetxt=""
targetdir="root"
filesizes="4770"
keep="y"
quiet="n"

print_cmd_arg=""
if type printf > /dev/null; then
    print_cmd="printf"
elif test -x /usr/ucb/echo; then
    print_cmd="/usr/ucb/echo"
else
    print_cmd="echo"
fi

unset CDPATH

MS_Printf()
{
    $print_cmd $print_cmd_arg "$1"
}

MS_PrintLicense()
{
  if test x"$licensetxt" != x; then
    echo $licensetxt
    while true
    do
      MS_Printf "Please type y to accept, n otherwise: "
      read yn
      if test x"$yn" = xn; then
        keep=n
 	eval $finish; exit 1        
        break;    
      elif test x"$yn" = xy; then
        break;
      fi
    done
  fi
}

MS_diskspace()
{
	(
	if test -d /usr/xpg4/bin; then
		PATH=/usr/xpg4/bin:$PATH
	fi
	df -kP "$1" | tail -1 | awk '{ if ($4 ~ /%/) {print $3} else {print $4} }'
	)
}

MS_dd()
{
    blocks=`expr $3 / 1024`
    bytes=`expr $3 % 1024`
    dd if="$1" ibs=$2 skip=1 obs=1024 conv=sync 2> /dev/null | \
    { test $blocks -gt 0 && dd ibs=1024 obs=1024 count=$blocks ; \
      test $bytes  -gt 0 && dd ibs=1 obs=1024 count=$bytes ; } 2> /dev/null
}

MS_dd_Progress()
{
    if test "$noprogress" = "y"; then
        MS_dd $@
        return $?
    fi
    file="$1"
    offset=$2
    length=$3
    pos=0
    bsize=4194304
    while test $bsize -gt $length; do
        bsize=`expr $bsize / 4`
    done
    blocks=`expr $length / $bsize`
    bytes=`expr $length % $bsize`
    (
        dd bs=$offset count=0 skip=1 2>/dev/null
        pos=`expr $pos \+ $bsize`
        MS_Printf "     0%% " 1>&2
        if test $blocks -gt 0; then
            while test $pos -le $length; do
                dd bs=$bsize count=1 2>/dev/null
                pcent=`expr $length / 100`
                pcent=`expr $pos / $pcent`
                if test $pcent -lt 100; then
                    MS_Printf "\b\b\b\b\b\b\b" 1>&2
                    if test $pcent -lt 10; then
                        MS_Printf "    $pcent%% " 1>&2
                    else
                        MS_Printf "   $pcent%% " 1>&2
                    fi
                fi
                pos=`expr $pos \+ $bsize`
            done
        fi
        if test $bytes -gt 0; then
            dd bs=$bytes count=1 2>/dev/null
        fi
        MS_Printf "\b\b\b\b\b\b\b" 1>&2
        MS_Printf " 100%%  " 1>&2
    ) < "$file"
}

MS_Help()
{
    cat << EOH >&2
Makeself version 2.2.0
 1) Getting help or info about $0 :
  $0 --help   Print this message
  $0 --info   Print embedded info : title, default target directory, embedded script ...
  $0 --lsm    Print embedded lsm entry (or no LSM)
  $0 --list   Print the list of files in the archive
  $0 --check  Checks integrity of the archive
 
 2) Running $0 :
  $0 [options] [--] [additional arguments to embedded script]
  with following options (in that order)
  --confirm             Ask before running embedded script
  --quiet		Do not print anything except error messages
  --noexec              Do not run embedded script
  --keep                Do not erase target directory after running
			the embedded script
  --noprogress          Do not show the progress during the decompression
  --nox11               Do not spawn an xterm
  --nochown             Do not give the extracted files to the current user
  --target dir          Extract directly to a target directory
                        directory path can be either absolute or relative
  --tar arg1 [arg2 ...] Access the contents of the archive through the tar command
  --                    Following arguments will be passed to the embedded script
EOH
}

MS_Check()
{
    OLD_PATH="$PATH"
    PATH=${GUESS_MD5_PATH:-"$OLD_PATH:/bin:/usr/bin:/sbin:/usr/local/ssl/bin:/usr/local/bin:/opt/openssl/bin"}
	MD5_ARG=""
    MD5_PATH=`exec <&- 2>&-; which md5sum || type md5sum`
    test -x "$MD5_PATH" || MD5_PATH=`exec <&- 2>&-; which md5 || type md5`
	test -x "$MD5_PATH" || MD5_PATH=`exec <&- 2>&-; which digest || type digest`
    PATH="$OLD_PATH"

    if test "$quiet" = "n";then
    	MS_Printf "Verifying archive integrity..."
    fi
    offset=`head -n 500 "$1" | wc -c | tr -d " "`
    verb=$2
    i=1
    for s in $filesizes
    do
		crc=`echo $CRCsum | cut -d" " -f$i`
		if test -x "$MD5_PATH"; then
			if test `basename $MD5_PATH` = digest; then
				MD5_ARG="-a md5"
			fi
			md5=`echo $MD5 | cut -d" " -f$i`
			if test $md5 = "00000000000000000000000000000000"; then
				test x$verb = xy && echo " $1 does not contain an embedded MD5 checksum." >&2
			else
				md5sum=`MS_dd "$1" $offset $s | eval "$MD5_PATH $MD5_ARG" | cut -b-32`;
				if test "$md5sum" != "$md5"; then
					echo "Error in MD5 checksums: $md5sum is different from $md5" >&2
					exit 2
				else
					test x$verb = xy && MS_Printf " MD5 checksums are OK." >&2
				fi
				crc="0000000000"; verb=n
			fi
		fi
		if test $crc = "0000000000"; then
			test x$verb = xy && echo " $1 does not contain a CRC checksum." >&2
		else
			sum1=`MS_dd "$1" $offset $s | CMD_ENV=xpg4 cksum | awk '{print $1}'`
			if test "$sum1" = "$crc"; then
				test x$verb = xy && MS_Printf " CRC checksums are OK." >&2
			else
				echo "Error in checksums: $sum1 is different from $crc" >&2
				exit 2;
			fi
		fi
		i=`expr $i + 1`
		offset=`expr $offset + $s`
    done
    if test "$quiet" = "n";then
    	echo " All good."
    fi
}

UnTAR()
{
    if test "$quiet" = "n"; then
    	tar $1vf - 2>&1 || { echo Extraction failed. > /dev/tty; kill -15 $$; }
    else

    	tar $1f - 2>&1 || { echo Extraction failed. > /dev/tty; kill -15 $$; }
    fi
}

finish=true
xterm_loop=
noprogress=n
nox11=n
copy=none
ownership=y
verbose=n

initargs="$@"

while true
do
    case "$1" in
    -h | --help)
	MS_Help
	exit 0
	;;
    -q | --quiet)
	quiet=y
	noprogress=y
	shift
	;;
    --info)
	echo Identification: "$label"
	echo Target directory: "$targetdir"
	echo Uncompressed size: 40 KB
	echo Compression: gzip
	echo Date of packaging: Thu May 14 14:37:56 UTC 2020
	echo Built with Makeself version 2.2.0 on 
	echo Build command was: "/usr/bin/makeself \\
    \"--notemp\" \\
    \"/src/work/udev_rules_installer/makeself/root\" \\
    \"/src/work/udev_rules_installer/makeself/st-stlink-udev-rules-1.0.2-4-linux-noarch.sh\" \\
    \"STM udev rules installer\" \\
    \"./setup.sh\" \\
    \"1.0.2-4\""
	if test x$script != x; then
	    echo Script run after extraction:
	    echo "    " $script $scriptargs
	fi
	if test x"" = xcopy; then
		echo "Archive will copy itself to a temporary location"
	fi
	if test x"y" = xy; then
	    echo "directory $targetdir is permanent"
	else
	    echo "$targetdir will be removed after extraction"
	fi
	exit 0
	;;
    --dumpconf)
	echo LABEL=\"$label\"
	echo SCRIPT=\"$script\"
	echo SCRIPTARGS=\"$scriptargs\"
	echo archdirname=\"root\"
	echo KEEP=y
	echo COMPRESS=gzip
	echo filesizes=\"$filesizes\"
	echo CRCsum=\"$CRCsum\"
	echo MD5sum=\"$MD5\"
	echo OLDUSIZE=40
	echo OLDSKIP=501
	exit 0
	;;
    --lsm)
cat << EOLSM
No LSM.
EOLSM
	exit 0
	;;
    --list)
	echo Target directory: $targetdir
	offset=`head -n 500 "$0" | wc -c | tr -d " "`
	for s in $filesizes
	do
	    MS_dd "$0" $offset $s | eval "gzip -cd" | UnTAR t
	    offset=`expr $offset + $s`
	done
	exit 0
	;;
	--tar)
	offset=`head -n 500 "$0" | wc -c | tr -d " "`
	arg1="$2"
    if ! shift 2; then MS_Help; exit 1; fi
	for s in $filesizes
	do
	    MS_dd "$0" $offset $s | eval "gzip -cd" | tar "$arg1" - $*
	    offset=`expr $offset + $s`
	done
	exit 0
	;;
    --check)
	MS_Check "$0" y
	exit 0
	;;
    --confirm)
	verbose=y
	shift
	;;
	--noexec)
	script=""
	shift
	;;
    --keep)
	keep=y
	shift
	;;
    --target)
	keep=y
	targetdir=${2:-.}
    if ! shift 2; then MS_Help; exit 1; fi
	;;
    --noprogress)
	noprogress=y
	shift
	;;
    --nox11)
	nox11=y
	shift
	;;
    --nochown)
	ownership=n
	shift
	;;
    --xwin)
	finish="echo Press Return to close this window...; read junk"
	xterm_loop=1
	shift
	;;
    --phase2)
	copy=phase2
	shift
	;;
    --)
	shift
	break ;;
    -*)
	echo Unrecognized flag : "$1" >&2
	MS_Help
	exit 1
	;;
    *)
	break ;;
    esac
done

if test "$quiet" = "y" -a "$verbose" = "y";then
	echo Cannot be verbose and quiet at the same time. >&2
	exit 1
fi

MS_PrintLicense

case "$copy" in
copy)
    tmpdir=$TMPROOT/makeself.$RANDOM.`date +"%y%m%d%H%M%S"`.$$
    mkdir "$tmpdir" || {
	echo "Could not create temporary directory $tmpdir" >&2
	exit 1
    }
    SCRIPT_COPY="$tmpdir/makeself"
    echo "Copying to a temporary location..." >&2
    cp "$0" "$SCRIPT_COPY"
    chmod +x "$SCRIPT_COPY"
    cd "$TMPROOT"
    exec "$SCRIPT_COPY" --phase2 -- $initargs
    ;;
phase2)
    finish="$finish ; rm -rf `dirname $0`"
    ;;
esac

if test "$nox11" = "n"; then
    if tty -s; then                 # Do we have a terminal?
	:
    else
        if test x"$DISPLAY" != x -a x"$xterm_loop" = x; then  # No, but do we have X?
            if xset q > /dev/null 2>&1; then # Check for valid DISPLAY variable
                GUESS_XTERMS="xterm rxvt dtterm eterm Eterm kvt konsole aterm"
                for a in $GUESS_XTERMS; do
                    if type $a >/dev/null 2>&1; then
                        XTERM=$a
                        break
                    fi
                done
                chmod a+x $0 || echo Please add execution rights on $0
                if test `echo "$0" | cut -c1` = "/"; then # Spawn a terminal!
                    exec $XTERM -title "$label" -e "$0" --xwin "$initargs"
                else
                    exec $XTERM -title "$label" -e "./$0" --xwin "$initargs"
                fi
            fi
        fi
    fi
fi

if test "$targetdir" = "."; then
    tmpdir="."
else
    if test "$keep" = y; then
	if test "$quiet" = "n";then
	    echo "Creating directory $targetdir" >&2
	fi
	tmpdir="$targetdir"
	dashp="-p"
    else
	tmpdir="$TMPROOT/selfgz$$$RANDOM"
	dashp=""
    fi
    mkdir $dashp $tmpdir || {
	echo 'Cannot create target directory' $tmpdir >&2
	echo 'You should try option --target dir' >&2
	eval $finish
	exit 1
    }
fi

location="`pwd`"
if test x$SETUP_NOCHECK != x1; then
    MS_Check "$0"
fi
offset=`head -n 500 "$0" | wc -c | tr -d " "`

if test x"$verbose" = xy; then
	MS_Printf "About to extract 40 KB in $tmpdir ... Proceed ? [Y/n] "
	read yn
	if test x"$yn" = xn; then
		eval $finish; exit 1
	fi
fi

if test "$quiet" = "n";then
	MS_Printf "Uncompressing $label"
fi
res=3
if test "$keep" = n; then
    trap 'echo Signal caught, cleaning up >&2; cd $TMPROOT; /bin/rm -rf $tmpdir; eval $finish; exit 15' 1 2 3 15
fi

leftspace=`MS_diskspace $tmpdir`
if test -n "$leftspace"; then
    if test "$leftspace" -lt 40; then
        echo
        echo "Not enough space left in "`dirname $tmpdir`" ($leftspace KB) to decompress $0 (40 KB)" >&2
        if test "$keep" = n; then
            echo "Consider setting TMPDIR to a directory with more free space."
        fi
        eval $finish; exit 1
    fi
fi

for s in $filesizes
do
    if MS_dd_Progress "$0" $offset $s | eval "gzip -cd" | ( cd "$tmpdir"; UnTAR x ) 1>/dev/null; then
		if test x"$ownership" = xy; then
			(PATH=/usr/xpg4/bin:$PATH; cd "$tmpdir"; chown -R `id -u` .;  chgrp -R `id -g` .)
		fi
    else
		echo >&2
		echo "Unable to decompress $0" >&2
		eval $finish; exit 1
    fi
    offset=`expr $offset + $s`
done
if test "$quiet" = "n";then
	echo
fi

cd "$tmpdir"
res=0
if test x"$script" != x; then
    if test x"$verbose" = xy; then
		MS_Printf "OK to execute: $script $scriptargs $* ? [Y/n] "
		read yn
		if test x"$yn" = x -o x"$yn" = xy -o x"$yn" = xY; then
			eval $script $scriptargs $*; res=$?;
		fi
    else
		eval $script $scriptargs $*; res=$?
    fi
    if test $res -ne 0; then
		test x"$verbose" = xy && echo "The program '$script' returned an error code ($res)" >&2
    fi
fi
if test "$keep" = n; then
    cd $TMPROOT
    /bin/rm -rf $tmpdir
fi
eval $finish; exit $res
‹ ÄW½^í[{sÛF’÷¿BU¾Ã„Ö(/I‘zz“({I¸P$ å¨$‡"àá!Y±}Ÿı~İ3 !‘½UZmvCTÙ03İ=ıî¨²õâ~Uqìí½¨Öj»{øİ>Ø?Øå÷úzQÛÙß;ØŞÛ9¨â}­º‹a±÷â®$ŠİPˆÓ ”÷óó®ü;ÛyñïvU¶ÆŞDF2Şú#É3Vò^ùïşµÅÏ¿¹­UÂïTşû»»Ÿ•ÿnmÿ‘ü÷¶kµ¢º’ÿ?üz)œn¿ıÓiÿÂê8v«)j•je»¼k¼Q<İÙ#/·2¼ƒÀGQIÜyñµ|~Ä¤-¥3XàÉ ïl_4*†áôœK§k;‡‡…$JÂìv;Îot!ıQ~ÂûêîëÜ@;FÉ0¦‘ƒİ]Œüb\ç­cë°°¿_Å›ÓN«×>,Ì&ÉÕHŞâE×<ı0¸Ã¡Œ"<[Í‹öqÿü¼l]Øu«oŸ6[ëÓa¡–t.Ïvó'¬Ku¾ÿ~Áxñ§»–ÙÿÎóÚm¯¶·hÿ»+ûÿ§Û?iCùvG[¾(‚UşÈ¾¸r:£‘m
Ï°ğIàd(¦ÁH«épÅ‘$W›<òaôŒaGíŠİÃ.Ÿ“ô§"zü/HôŞÎóıoâÿ·Ÿ;ÿÛ^’ÿí¬üÿ¿Rş·Ïÿú÷d÷IòÀ×ÏhÉÛ²<p©ı—Ÿ¶ü’ıl?®ÿ÷vv÷Wöÿ‡°?Ndğã/?(ÿNªOkÿƒç´ÿrí©íÙ¿ıÏÂ`:‹û “¼ÇÿCéG²]?qÿïóö¿]=ØİY°ÿjueÿÏbÿßn<kàF×†áÅ[QXo@ëšÕ7Ë<¾ì›õºÕîZÇñN|/âkékò½‹ª1öÂlâŞ§ºSÜŒ¡‹~VëÄ€'ğ†a 'r‡ï#áãøÎ¥h¨%Â¼
¥œJ?şÆøÆp&rÂ×¢#ow·Îİpx-¶«µ×4ÔHÓ=äƒ÷A
²œJÍ¡A(š­f9÷fp/"FíùW¢`µ—ÍÄÓqó»ÙK7Ê@Ä@N‚;ªl±a1•#Ï­	G—Ân:]³<õVû¿%@zÓl´Ìc~ xCã­huÏ¬ÎÛ±D_uÏlG8­“î³c‰¶YÿÉ<µh¢Ù¼Äc§+h8'Šfó˜DÇj˜à?Ô{çV³kvá©7ÅI§uNÖ®wZVÃªw;­¦]w@_×ê4y’ÙÍÊÜ1pÄQÇlÖÏ€èxøì®#Ì“»a3ğzë¼m6mËÅE˜›%MHİnÛ  $*¬3³q"@é™}îXtÛ nK¼õüÚVÚİK¾9³A‰ÓÃPÆXçíFëäh"­æ)ØƒÇÓÒº-@G­8(Ëy©õW­!~±àz>5'µq2å„^$îDÀÎdß‹Ğ»º#ÕÈH¿bXÄpg3è¹;˜H¡ôô*–á4cè”ëßC_¼pT¹$J5İó‡A8BV-V*/šÎÜá{¥º&9­™ôa,IEî2"Tt$Ç(×+YK!RÒJÅ¡7Hb/ğKx©€„'°í -C¤7°}©=Hğ`Ì`¨Òû¢Ñ„Pã4ù7Hbêíxc2FÍËA±Ó~AÆ­7ÂM|í2 œ€nâİ0€ÅÑ¢ˆMeüİ7F­ëÏo†hÓd©½TTè $øA¼)¦päØqìz>S;f÷,B†¤JŠå %`y´şèMTóL\o*ChÎöcZÈIäWò=ùriKŒt*SR5`‚¦A¿B¢•_ìñˆ ´‰©ë'c
’ã¹§‰¤MK\'Éb.¡d6"7Hs£~R+¥ÌPâ—¼;w€*î‰¸’zÅQ0LÈk»©†m€T;‚Qxî$šŸ5†—å˜	ïTDSz¼ŠF}w*‰–%›÷ƒùVW…‹9Lğ‚0"]n[S%¬$Ñ$Îá"VnÊ€°Q[YXèİbÚŸFËrö™A4“C²q’îûÊ
¢Hmu·"ºŸ·hbğ“dD¬ÏW”2š©Ä+ˆõ.o´ı.®õ!İ<·nßËa“­Oää^¿¶ĞğLÇRSÒt–/ËÏ]¦Ôîrµ¾^¯Á›=¨A@>r\˜.ø‚åB!öAç&Œ(b¾ÕrQ×Zûå³¶ÁİÉ$çá—Â%¿e¿ï•'ÆFö+˜†·Ÿ“+—+$ñÑe—¬s¡ækĞå£qVºğì
RìÅBNšy2 lœ„lAZkÓĞ´H,¶D@—Ä±„ÈÀò„ƒ,û›ßc¤oÙsœß0-‹³ñœ/urÉA‘TDanAD×9‚ˆ«Ğ7Rw×vÊÿM¼*
.°ÕÇbÀÑ;ÌÂV>ÑâI”ãCcHM£$;ª;/"©ÓéÈ­ëMˆ7%\B%£oèEë4l‘˜Íi¬Ì¯œ4I¿ïî*„@Á¯á•Ê2ˆ;Ú{NÁ Ô2Bx®Ênä{$HqÚì‰S	B¾ÔN˜šeòÅÓvºe'ŞG-5¡U×r¡(‰#ç8CLTÛİô†uğ9ïI–„Ph)U
&¥`ßÏ óÌü¢Æç3sê™tTÂGáù3Çùh[YJÃ¢­(ZàIÌäè[ŒÈªtkvßüáOG[ü š4ş@W•İ.)*ãz3€p©‚ãè_;Ÿı=oîáXyCyEı#bíçXy¿®,K®]âˆ XW*0ŒN2´w§„Ş§ò¯ÄÜÈ¼C,)M^êã¾1şJ¶D<Åë;Î%”„€!d!s^5,O\Xq¤Y¼Cá™³!“œc÷FíÂUÑıÀ¯2 sFŞo şk o0=¡‡l8Ì
ŠÓCvz_¿ıŒög9-N<äñPX½dn:s-CÊ^}¼'å‹ÙkİëäÏùÊ½SÄC’IšÜŒÇºZ'ÔT±üeI.ebÎqÁòItŞB¨Ub‘"¦,ViÑôuÇ5Nj)E¹z(’^”noo4W3é,’hOĞÉv(Ó--‰E8\é.¯Šñ®İi]ØÇ—ÎKúÔn¨·šİ}Ôë¶:(˜–x€Ë÷ŸÛËqJÂF•n
t§kvi6Jz³Ó1Qà[4¡YoôT3°¸ÉÒ°áÍ±¦ÛR½„Æ|u	Î­Nıæ‘İ°»—%qbw›À)N¨CÂı»Şk˜ÑîuÚ-ªòA5}ìæI¹à'HàBç˜\rG¤A;í™âBÛ¢·ûôŒz `Ò±íÔ¦}Îô1y'=,pºØs— bÈï*Ş5Ì7ÀZa]Ğ¨sf6ËX
¢pôˆºæQÃRûOí¦Çæwu	, c´¶U·éÆú™:#fç²¤a:Öÿô¨‚›ç°#Š_à:6^ïu2Q‡»kCx–8mµ™XÇêPÏÙù^4Z¤çX%`èšŒ  ãşˆúY$î7uzmÕ•:k½S@£‰¥ÜÇ¡>ml%=Pâ–ï›3‹ú]ÄNæ”I,pÀ±z7?øÀÀnn¢i6ìS«Y·´Ù6!P›;m¶BûÆÎÖ	êÂYZ=¤Äòö‰0/l"_/‚¦9¶¢E±®~¦Ù‹³~¦ö¡€±hi\æô…šy½æ±ÕaV€éÿ)§Zf¶auVÕµâ!Ö:™‚°—•gô‚-¨‡;wØ'ÁZ°xf}ûqZ½¸¤à‚Ì9ÖépÚ/K¹¨c.½-š ­¶ÄVsJ†ÒíQÿ’­%ƒô»†GŒ\j2Ù¶•ù‹['Æ'Ãx)Î)>F¨'ÅŠÔ¹Å—ß¦uU‡H|Ê?ÈyR5½Y¼™ö·‹³H”ŠLSq-Ê3±¾¾YßŠ¶!Şi[EÖÇÂjÿıW»Ş: ”¹lÈ÷¿‰Ê6Ÿ©}ªàÍrÚ¬1]Á¬c‰|›ƒÕ³yßEú·b;µVJbxQša¤Èó‚aFil˜¨¯ˆ”HPÉ+c„Rê¼`;"L|•±ºš%,¢\CYFd¤f "n,r7SÇ 7KÂYÀÕ@!‹¢]E•¡ÊÁ Û4ŒH‚+àÂ xß½x"|XO~M”»şÃH7ÑWÇQ|ˆ°ût:}¸^3Ö&„)Â¡a¬Å÷3ÉX%ıèN†ÆÚ#ˆ‚Îs Bpp¤;"9gıQ¼ß[âÑ€(Â}½mnİ¿…²ÖõX‡¢p_ïÒcµ—Ùn”lå/Ãáa?„\W
 k=É/&‰=PË ¼zõ*›GÄf€D«r¹}–:‹3{¡IÈÉ1ÕVæƒ±FÚÔö˜T1;t}­4Cw–µk”ù”ØÎ8Ëˆåt&H$_´ˆÀÈE±„©Æ€øQ¤Ã)ãÃ;˜òÜ|IÌÇ¡ÒApL
ÂÚØiŸ³¿ëLç¬¯ÜÚÛê»OâğÿÄÆÖ­nÅÓÙV8›–ñ[%g‚Òär¨Bˆ±/,’å‰Ù€ß›Ÿj)¶øÌVĞE9ø­ªD‘  ü^fåÔ¯øs­–²Ÿœt™ 'ku¯(_c¾2ÒĞMFA¸[ÒvZñk‹âˆFt¸şAß}Wş®ú)%¡]o•_)¤I+|E µ7>dA¼	**(HÿHæäDGÁßr
ÙiÔ‡ŠGg­`fn“Ã	œ@Ä°ä˜æÁ«ø¬{ìR&2*¼xÎê[#y»Ç÷,~Ò –B9Uò¿…uø…>Â\İêSBÓjX…Ì÷¾ä»µÌÑiƒë³…,(ôüÚ_4Ì*øãn‹ÂQrE5ÎzU­Ù,h…ÜaÒ^R÷1¢	ı4j)Ì)›ÃNuB+DEÔ•¥>:ÍÉ…9 ffŠr›Qç‰a½0Šç=!AzEşİÁ‚«>İG‡…ß¤ï¡Â|OD"q‹¥í.?Ñ¬îEA~RüÈBğ”0¶u^²™ÙªZxÈ¯ñ8 oXZÔ.5Œ!¶jè@d0îMf¶.èç‡{«Š:Ç› ¼qCî@øı\#µs*”%‡a¨laî“útÛ‡ëPœŞûv«ÌÁÊM‘ıJS£ğ­ÕöŞïBÕ¢şMD­ZÅV<R±¾ˆ®@Á8é{É0ˆ Kr>níûï%2fX¥ÈEm9¼ÑgËª›ë&¶ÌX5µ”aæÁÍ†ª%ë½vğÑCJ!&ìâWjïGwîLÃA9èEæ
ÀndEdòi³H…÷\6²$‚jJ~^”Ë±|—=¤Ï*²ç™¯î¼Q|}¸¿V—¯%Õ©‡{à»Tt—™ºüi¼næ#NæiÂÇJEªiO+‰ÍÁóÙÈ±Ñ/L{›Çé¯±©%×XÎ¯6ó~D›[Û˜m ¾Ê/ÈÈr¼æs	:³ÉÜÚwœş.Líİ¾ğı<W2{Ú~–şı_ísßÿm×¶wşşoõıßóÿCÙÅÈá÷(S¤ËDn®ˆw|*»^İD‚O®.„²rÑ×‹òëd“j¨ê<9ëë¹>ƒ€}˜¶(•ÎdGLæPË%‹5æß!e8ø”&ª'	7¼šƒ|'=‚OEÔÊ˜è Š*ÈÒrk¿uÍšÏ}-§ùğ7Qö%±ÀÈ}%¥°üEmæÂ;nÉx¸•P\æ—•ÑWM¿&`øú‡ùÄO¯ŒèÚÇg}`®+ŒÁ‘ù@):U¤2>&$Îì€¶é›_*-%‚=»STÅÅ‡ƒràªó$’–æl”ë²Rc–VU\š#à¯fdzØF•õ‘©ı¼9âÌ?ù#áå°¨‘~¦YEøeú§k	ı^¸wĞÉ±ñVü¾ûËÒş‡4o}˜…HŞÅúÎ§ÇÌ\[Û5öD8M€±¶Éqä!"Ê\0µ\ºF^öµE^tq±şh9l!¯±Y‰‘÷â1·+èS¼æ>†¤zçÈ&Ôî%vSÀ”Ä¦ßâ¥I6g•Ü5©Ã¢Ä¦¦W>¦œ²|¥§bw—òóõ9ÛæªŸ~(şÊÈE¢vzB’Š¾ògü+ÊÕµºV×êZ]«ku­®ÕµºV×êZ]«ë}ı?ış	‰ P  