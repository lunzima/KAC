/* kac -- KAC encoder and decoder command line.
 * Encoding: kac [options] in.wav [out.kac]
 * Decoding: kac --decode [options] in.kac [out.wav] */
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "kac.h"
#include "wav_io.h"

#define KAC_CHUNK_FRAMES 4096u

/* The command line stops at level 9; level 10 is a library-only setting. */
#define KAC_CLI_QMAX 9u

/* --preset maps to -V indices: low 8, medium 6.5, standard the default rung,
 * extreme 3, insane 2. Values are thousandths; rung 0 is not a preset. */
#define KAC_PRESET_LOW 8000
#define KAC_PRESET_MEDIUM 6500
#define KAC_PRESET_STD (KAC_DEFAULT_V * 1000)
#define KAC_PRESET_EXTREME 3000
#define KAC_PRESET_INSANE 2000

/* Encoding-only options, refused with --decode in either order. */
#define KAC_SEEN_B 0x0001u
#define KAC_SEEN_BMAX 0x0002u
#define KAC_SEEN_V 0x0004u
#define KAC_SEEN_PSNR 0x0008u
#define KAC_SEEN_ABR 0x0010u
#define KAC_SEEN_ABRBPS 0x0020u
#define KAC_SEEN_TIER 0x0040u
#define KAC_SEEN_QUAL 0x0080u
#define KAC_SEEN_MS 0x0100u
#define KAC_SEEN_STRIDE 0x0200u
#define KAC_SEEN_LIMIT 0x0400u
#define KAC_SEEN_LOOP 0x0800u
#define KAC_SEEN_MONO 0x1000u
#define KAC_SEEN_LOWDELAY 0x2000u
#define KAC_SEEN_BRUTE 0x4000u

typedef struct {
    /* input, output and action */
    const char *in, *out;
    int         decode;
    int         silent;

    /* quality and rate selection */
    int    have_v;    /* -V 0 is legal, so the index alone cannot say */
    int    v;         /* -V index in thousandths of a rung, 0..9999 */
    int    def_v;     /* nothing selected: the encoder's default rung applies */
    double psnr;      /* --psnr target, dB; 0 = unused */
    double abr;       /* --abr target, kbit/s; 0 = unused */
    double abr_bps;   /* --abr-bps target, bit/channel*sample; 0 = unused */
    double brate;     /* -b: bit/channel*sample */
    double brate_max; /* -B: ceiling of the per-block tier, same unit; 0 = unused */
    int    tier;
    int    have_quality; /* -q 0 is legal, so the level alone cannot say */
    int    quality;      /* -q, 0..KAC_QUALITY_MAX */
    int    brute;        /* --brute 1|2; 0 = off */

    /* channel form */
    int ms_mode; /* -m s|j|f: KAC_MS_*, the default is AUTO */
    int mono;    /* --mono / -a: average a stereo input to one channel */

    /* loop, seek index and length limits */
    uint32_t loop_start, loop_end;
    uint32_t stride;
    int      have_limit;
    uint32_t limit;
    int      have_loops; /* --loop-count 0 is legal on a decode */
    uint32_t loops;

    /* low-delay stream */
    int      lowdelay;            /* --lowdelay: a low-delay stream */
    int      lowdelay_tier;       /* fixed tier when no VBR setting is given, 1..7 */
    int      lowdelay_tier_named; /* the tier came from --lowdelay's token */
    int      have_chunk_ms;
    uint32_t chunk_ms; /* --chunk-ms target, default 5 */

    /* which encoding options were given, for conflict checks and warnings */
    unsigned seen;    /* KAC_SEEN_* */
    int      v_named; /* -V given (--preset is tracked separately) */
    int      preset_named;
    int      q_named;       /* -q given */
    int      fh_named;      /* -f or -h given */
    int      m_named;       /* -m given */
    int      mono_named;    /* -a or --mono given */
    int      ms_auto_named; /* the last -m was j (the measured form) */

    /* diagnostics and derived output */
    char err[160];     /* why parse_opts failed, for main */
    char outbuf[1024]; /* derived output name when none was given */
} kac_opts;

/* Whether the command line selected a per-block quality or rate setting. */
static int opts_named_setting(const kac_opts *o)
{
    return o->have_v || o->psnr > 0.0 || o->abr > 0.0 || o->abr_bps > 0.0;
}

/* The rung index in thousandths, the unit kac_set_vbr() takes for -V: -V 3.5 is
 * 3500 and the ladder's last rung, -V 9.999, is 9999. */
static int opts_rung1000(const kac_opts *o)
{
    return o->have_v ? o->v : KAC_DEFAULT_V * 1000;
}

/* How both directions describe the stream under the banner: the rate in kHz and
 * the channel count as a word. */
static void stream_desc(char *buf, size_t n, unsigned rate, unsigned ch)
{
    snprintf(buf, n, "%.4g kHz %s", (double)rate / 1000.0, ch == 1 ? "mono" : "stereo");
}

/* Summary line: head tier, rate and frame count. */
static void summary_line(const kac_opts *o, unsigned tier, int per_block, double brate,
                         uint32_t frames, unsigned rate, unsigned ch)
{
    fprintf(stderr, "%s -> %s  T%u%s %.4f bit/sample  %u frames %u Hz %u ch\n", o->in, o->out, tier,
            per_block ? " (head)" : "", brate, frames, rate, ch);
}

/* Progress uses CPU time, not wall time. */
#define KAC_PROGRESS_SEC 0.1 /* redraw at most this often */

typedef struct {
    uint64_t total, done;
    unsigned rate; /* sample rate: the speed column is audio seconds per CPU second */
    clock_t  t0;
    double   last;
    int      on; /* 0 when silent: every entry point below then does nothing */
} kac_progress;

static void progress_time(char *buf, size_t n, double sec)
{
    long t = (long)(sec + 0.5);
    if (t < 0)
        t = 0;
    if (t > 359999L) /* 99:59:59, well past any encode or decode this tool runs */
        t = 359999L;
    snprintf(buf, n, "%ld:%02ld", t / 60, t % 60);
}

static void progress_draw(kac_progress *p, int force)
{
    double el    = (double)(clock() - p->t0) / (double)CLOCKS_PER_SEC;
    double speed = 0.0, est = 0.0, eta = 0.0;
    char   e[24], t[24], x[24];
    if (!p->on)
        return;
    if (p->done && el > 0.0 && p->rate)
        speed = (double)p->done / ((double)p->rate * el); /* play/CPU, as a ratio */
    if (speed > 0.0)
        est = (double)p->total / ((double)p->rate * speed);
    eta = est > el ? est - el : 0.0;
    if (!force && el - p->last < KAC_PROGRESS_SEC)
        return;
    p->last = el;
    progress_time(e, sizeof e, el);
    progress_time(t, sizeof t, est);
    progress_time(x, sizeof x, eta);
    /* The columns match progress_header() character for character. */
    fprintf(stderr, "\r%9llu/%-9llu(%3u%%)| %5s/%5s| %8.2fx| %6s   ", (unsigned long long)p->done,
            (unsigned long long)p->total, (unsigned)(p->total ? p->done * 100u / p->total : 0u), e,
            t, speed, x);
    fflush(stderr);
}

static void progress_header(const kac_progress *p)
{
    if (!p->on)
        return;
    fprintf(stderr, "%9s/%-9s(%4s)| %5s/%5s| %8s | %6s\n", "Frame", "total", "%", "CPU", "estim",
            "play/CPU", "ETA");
}

static void progress_init(kac_progress *p, uint64_t total, unsigned rate, int on)
{
    p->total = total;
    p->done  = 0;
    p->rate  = rate;
    p->t0    = clock();
    p->last  = -KAC_PROGRESS_SEC;
    p->on    = on;
    progress_header(p);
    progress_draw(p, 0);
}

static void progress_set(kac_progress *p, uint64_t done)
{
    p->done = done;
    progress_draw(p, 0);
}

static void progress_finish(kac_progress *p)
{
    p->done = p->total;
    progress_draw(p, 1);
    if (p->on) {
        fputc('\n', stderr);
        p->on = 0; /* so a later error path knows the line was already closed */
    }
}

/* Closes a progress line left open by an early exit, so the error message is not
 * appended to it. Safe on a handle that never drew anything. */
static void progress_abandon(kac_progress *p)
{
    if (p->on) {
        fputc('\n', stderr);
        p->on = 0;
    }
}

static void usage_head(FILE *f)
{
    fprintf(f,
            "%s\n\n"
            "usage: kac [options] <infile> [outfile]          encode a WAV to KAC\n"
            "       kac --decode [options] <infile> [outfile] decode a KAC to WAV\n"
            "       outfile defaults to infile with .kac (encode) or .wav (--decode).\n",
            kac_version());
}

static void usage_medium(FILE *f)
{
    usage_head(f);
    fprintf(f, "\nRECOMMENDED:\n"
               "    kac in.wav out.kac            default, -V 5\n"
               "    kac -V 0 in.wav out.kac       best quality\n"
               "    kac -V 8 in.wav out.kac       --preset low\n"
               "    kac -V 6.5 in.wav out.kac     --preset medium\n"
               "    kac -V 5 in.wav out.kac       --preset standard\n"
               "    kac -V 3 in.wav out.kac       --preset extreme\n"
               "    kac -V 2 in.wav out.kac       --preset insane\n"
               "    kac --tier 3 in.wav out.kac   one fixed tier\n"
               "\nOPTIONS:\n"
               "\n  Quality and rate:\n"
               "    -b <rate>        bit/channel*sample: pinned tier, or the per-block\n"
               "                     floor with a VBR mode\n"
               "    -B <rate>        per-block ceiling; ignored when the tier is fixed\n"
               "    --tier <2..6>    one tier for every block\n"
               "    --psnr <dB>      VBR, target PSNR, bit rate floats\n"
               "    -V <0..9.999>    VBR, quality target, bit rate floats;\n"
               "                     0 = best, default 5\n"
               "    --preset <name>  low|medium|standard|extreme|insane\n"
               "    --abr <kbps>     VBR, average bit rate, quality floats\n"
               "    --abr-bps <rate> the same in bit/channel*sample\n"
               "    --lowdelay [tier] low-delay stream: tier (default 3),\n"
               "                     chunks near 5 ms, at most 5.5 ms\n"
               "    --chunk-ms <ms>  with --lowdelay: target chunk length, default 5\n"
               "\n  Operational options:\n"
               "    --loop <S,E>     loop region in samples\n"
               "    --stride <N>     seek index every N samples (multiple of the block)\n"
               "    --limit <N>      encode the first N samples per channel\n"
               "    -m <s|j|f|m>     s = L/R, j = measured (default), f = mid/side,\n"
               "                     m = mono\n"
               "    --mono, -a       downmix to one channel\n"
               "    -q <0..9>        algorithm quality, 0 = best, default 3\n"
               "    -f               -q 7\n"
               "    -h               -q 2\n"
               "    --brute <1|2>    a harder search than any -q level\n"
               "    --decode         decode instead of encode\n"
               "    --loop-count <K> with --decode: play the loop K times\n"
               "\n  Other:\n"
               "    -S, --silent, --quiet\n"
               "                     no banner, progress line or summary\n"
               "    --help           this list\n"
               "    --longhelp, -?   this list and the notes\n"
               "    --version        version\n");
}

static void usage_long(FILE *f)
{
    usage_medium(f);
    fprintf(f, "\nNOTES:\n"
               "  -V and --preset set the same rung; the later one wins, with a warning.\n"
               "  --psnr and --abr/--abr-bps are separate modes and are refused with\n"
               "  -V/--preset and with each other.\n"
               "  -b is a rate: alone it pins a tier in T2..T6; with a VBR mode it is\n"
               "  the floor and -B the ceiling, over T1..T7. --tier is refused with a\n"
               "  VBR mode or -b.\n"
               "  -m j measures the channel form per block; -m s and -m f force one.\n"
               "  --lowdelay makes a stream: default T3, mid/side, -q 7, chunks near\n"
               "  5 ms (at most 5.5 ms). All encoder options apply except --loop and\n"
               "  --stride. The chunk length is in the stream header; the last chunk is\n"
               "  padded, so the output can hold up to N-1 extra samples.\n"
               "  --brute replaces the -q level: 1 and 2 are settings past -q 0.\n"
               "  The banner, progress line and summary go to stderr; -S suppresses\n"
               "  them. --mono is an encode option and is refused with --decode.\n");
}

/* A decimal uint32 and nothing else. A sign is refused rather than folded:
 * a folded -1 would read as "no limit". */
static int parse_u32_span(const char *s, const char **end, uint32_t *v)
{
    uint64_t    x = 0;
    const char *p = s;

    if (!p || !*p)
        return -1;
    while (*p >= '0' && *p <= '9') {
        x = x * 10u + (uint64_t)(*p - '0');
        if (x > 0xffffffffu)
            return -1;
        p++;
    }
    if (p == s)
        return -1;
    *end = p;
    *v   = (uint32_t)x;
    return 0;
}

static int parse_u32(const char *s, uint32_t *v)
{
    const char *e;

    return (parse_u32_span(s, &e, v) == 0 && *e == '\0') ? 0 : -1;
}

/* A finite positive number with nothing after it. strtod() answers with
 * infinities and NaN for some inputs and accepts a trailing suffix, and neither
 * is a rate or a level. */
static int parse_positive(const char *s, double *v)
{
    char  *e;
    double d;

    if (!s || !s[0])
        return -1;
    d = strtod(s, &e);
    if (*e || !isfinite(d) || !(d > 0.0))
        return -1;
    *v = d;
    return 0;
}

/* A legal configuration that does something other than what it looks like. The
 * run goes ahead; the line goes to stderr whether or not -S is on, because it is
 * not the banner. */
static void warning(const char *fmt, ...)
{
    va_list ap;

    fputs("kac: warning: ", stderr);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    fflush(stderr);
}

/* A tier's nominal rate: 64 bits over its slice length. */
static double tier_nominal_brate(int tier)
{
    return 64.0 / (double)kac_tiers[tier - 1u].n;
}

/* Whether a rate lies outside a tier span, and which end the lookup will hold
 * it at. 0 means it lies inside. */
static int rate_outside_window(double r, int *end_tier, int lo, int hi)
{
    if (r < tier_nominal_brate(lo)) {
        *end_tier = lo;
        return -1;
    }
    if (r > tier_nominal_brate(hi)) {
        *end_tier = hi;
        return 1;
    }
    return 0;
}

/* The value of a short option: attached (-q3, -ms) or the next word. *i moves
 * only in the second case. */
static const char *short_value(int argc, char **argv, int *i, const char *a, size_t len)
{
    if (a[len] != '\0')
        return a + len;
    if (*i + 1 < argc) {
        (*i)++;
        return argv[*i];
    }
    return NULL;
}

/* The output name derived when only the input is named: a trailing .wav becomes
 * .kac (and .kac becomes .wav); any other name gets the extension appended.
 * Returns 0, or -1 if the name does not fit. */
static int derive_out(kac_opts *o)
{
    const char *src = o->decode ? ".kac" : ".wav";
    const char *dst = o->decode ? ".wav" : ".kac";
    size_t      n   = strlen(o->in);
    size_t      cut = n;
    size_t      i;

    if (n >= 4) {
        int same = 1;
        for (i = 0; i < 4; i++) {
            if (tolower((unsigned char)o->in[n - 4 + i]) != src[i])
                same = 0;
        }
        if (same)
            cut = n - 4;
    }
    if (cut + strlen(dst) + 1 > sizeof o->outbuf)
        return -1;
    memcpy(o->outbuf, o->in, cut);
    strcpy(o->outbuf + cut, dst);
    o->out = o->outbuf;
    return 0;
}

/* Record why the command line was refused. parse_opts() has the option text in
 * hand; main() prints the reason and exits 2. */
static int parse_fail(kac_opts *o, const char *opt)
{
    snprintf(o->err, sizeof o->err, "invalid or missing value for %s", opt);
    return -1;
}

/* The same, for a token that names no option at all. */
static int parse_unknown(kac_opts *o, const char *opt)
{
    snprintf(o->err, sizeof o->err, "unknown option %s", opt);
    return -1;
}

/* The same, for a failure about the combination of options rather than one
 * option's value. */
static int parse_error(kac_opts *o, const char *msg)
{
    snprintf(o->err, sizeof o->err, "%s", msg);
    return -1;
}

static int parse_opts(int argc, char **argv, kac_opts *o)
{
    int i;
    for (i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!strcmp(a, "-f")) {
            /* The two quick levels: -f is the fast one and -h is the one two
             * steps above the default. Both are points on the -q axis. */
            o->quality      = 7;
            o->have_quality = 1;
            o->fh_named     = 1;
            o->seen |= KAC_SEEN_QUAL;
        } else if (!strcmp(a, "-h")) {
            o->quality      = 2;
            o->have_quality = 1;
            o->fh_named     = 1;
            o->seen |= KAC_SEEN_QUAL;
        } else if (!strcmp(a, "--help") || !strcmp(a, "--usage")) {
            usage_medium(stderr);
            exit(0);
        } else if (!strcmp(a, "-?") || !strcmp(a, "--longhelp")) {
            usage_long(stderr);
            exit(0);
        } else if (!strcmp(a, "--version")) {
            fprintf(stderr, "%s\n", kac_version());
            exit(0);
        } else if (!strcmp(a, "-S") || !strcmp(a, "--silent") || !strcmp(a, "--quiet"))
            o->silent = 1;
        else if (!strcmp(a, "--chunk-ms")) {
            uint32_t v;
            if (i + 1 >= argc || parse_u32(argv[++i], &v) || v == 0u || v > 1000u)
                return parse_fail(o, a);
            o->chunk_ms      = v;
            o->have_chunk_ms = 1;
        } else if (!strcmp(a, "--decode"))
            o->decode = 1;
        else if (!strncmp(a, "-b", 2)) {
            const char *v = short_value(argc, argv, &i, a, 2);
            if (!v || parse_positive(v, &o->brate))
                return parse_fail(o, "-b");
            o->seen |= KAC_SEEN_B;
        } else if (!strncmp(a, "-B", 2)) {
            /* The ceiling of the same axis. A rate past the top tier's is held
             * at the top tier by the lookup rather than refused, so only a value
             * that is not a rate is an error here. */
            const char *v = short_value(argc, argv, &i, a, 2);
            if (!v || parse_positive(v, &o->brate_max))
                return parse_fail(o, "-B");
            o->seen |= KAC_SEEN_BMAX;
        } else if (!strcmp(a, "--lowdelay")) {
            /* The optional tier is the fixed tier unless a VBR setting, -b or
             * -B makes the run per-block. A non-number is the input file; a
             * number outside 1..7 is a bad tier. */
            uint32_t v;

            o->lowdelay      = 1;
            o->lowdelay_tier = 3;
            o->seen |= KAC_SEEN_LOWDELAY;
            if (i + 1 < argc && parse_u32(argv[i + 1], &v) == 0) {
                if (v < KAC_TIER_MIN || v > KAC_TIER_MAX)
                    return parse_error(o, "--lowdelay takes a tier from 1 to 7");
                o->lowdelay_tier       = (int)v;
                o->lowdelay_tier_named = 1;
                i++;
            }
        } else if (!strcmp(a, "--tier")) {
            uint32_t v;
            /* T1 and T7 are the two a whole file is not meant to sit on; the
             * bounds are kac_internal.h's. */
            if (i + 1 >= argc || parse_u32(argv[++i], &v) || v < KAC_TIER_FIXED_MIN ||
                v > KAC_TIER_FIXED_MAX)
                return parse_fail(o, a);
            o->tier = (int)v;
            o->seen |= KAC_SEEN_TIER;
        } else if (!strncmp(a, "-m", 2)) {
            /* s leaves the block as it arrived, j measures both forms (the
             * default), f forces mid and side, m encodes one channel. The last
             * -m wins and clears the others. */
            const char *m = short_value(argc, argv, &i, a, 2);

            if (!m || m[0] == '\0' || m[1] != '\0')
                return parse_error(o, "-m takes s, j, f or m");
            o->mono          = 0;
            o->ms_mode       = KAC_MS_AUTO;
            o->m_named       = 1;
            o->ms_auto_named = (*m == 'j');
            o->seen |= KAC_SEEN_MS;
            if (*m == 's')
                o->ms_mode = KAC_MS_LR;
            else if (*m == 'j')
                o->ms_mode = KAC_MS_AUTO;
            else if (*m == 'f')
                o->ms_mode = KAC_MS_MS;
            else if (*m == 'm')
                o->mono = 1;
            else
                return parse_error(o, "-m takes s, j, f or m");
        } else if (!strcmp(a, "--mono") || !strcmp(a, "-a")) {
            /* -a is the short spelling; --mono is the name that says what it does.
             * It is an encode-side thing: a decode writes what the file holds. */
            o->mono       = 1;
            o->mono_named = 1;
            o->seen |= KAC_SEEN_MONO;
        } else if (!strncmp(a, "-q", 2)) {
            uint32_t    v;
            const char *arg = short_value(argc, argv, &i, a, 2);
            if (!arg || parse_u32(arg, &v))
                return parse_fail(o, "-q");
            if (v > KAC_CLI_QMAX)
                return parse_error(o, "-q takes a level from 0 to 9");
            o->quality      = (int)v;
            o->have_quality = 1;
            o->q_named      = 1;
            o->seen |= KAC_SEEN_QUAL;
        } else if (!strcmp(a, "--brute")) {
            uint32_t v;
            if (i + 1 >= argc || parse_u32(argv[++i], &v) || v < 1u || v > 2u)
                return parse_fail(o, a);
            o->brute = (int)v;
            o->seen |= KAC_SEEN_BRUTE;
        } else if (!strcmp(a, "--psnr")) {
            /* Above 14-bit PCM's audio-band SNR a target is a typo or a unit
             * mistake, so it is refused here. */
            if (i + 1 >= argc || parse_positive(argv[++i], &o->psnr))
                return parse_fail(o, a);
            if (o->psnr > (double)KAC_AIM_MAX_MDB / 1000.0)
                return parse_error(o, "--psnr must be at most 85 dB");
            o->seen |= KAC_SEEN_PSNR;
        } else if (!strcmp(a, "--abr")) {
            uint32_t kbps;
            if (i + 1 >= argc || parse_u32(argv[++i], &kbps) != 0 || kbps == 0)
                return parse_fail(o, a);
            o->abr = (double)kbps;
            o->seen |= KAC_SEEN_ABR;
        } else if (!strcmp(a, "--abr-bps")) {
            double d;
            if (i + 1 >= argc || parse_positive(argv[++i], &d) || d > 1000.0)
                return parse_fail(o, a);
            /* The same mode as --abr, in the axis the ladder is built on. A goal
             * outside the window is held at the nearer end; only a value that is
             * not a rate is refused, and 1000 bit/sample is past every rate. */
            o->abr_bps = d;
            o->seen |= KAC_SEEN_ABRBPS;
        } else if (!strncmp(a, "-V", 2)) {
            char       *e;
            const char *arg = short_value(argc, argv, &i, a, 2);
            double      d;

            if (!arg || !arg[0])
                return parse_fail(o, "-V");
            d = strtod(arg, &e);
            /* The -V index is refused outside 0..9.999, never clamped: "-V 12"
             * silently meaning -V 9 would be a downgrade. Three decimals are
             * accepted; -V 9.999 is the ladder's last rung. */
            if (*e || !isfinite(d) || !(d >= 0.0) || d > 9.999)
                return parse_fail(o, "-V");
            o->v       = (int)(d * 1000.0 + 0.5);
            o->have_v  = 1;
            o->v_named = 1;
            o->seen |= KAC_SEEN_V;
        } else if (!strcmp(a, "--preset")) {
            if (i + 1 >= argc)
                return parse_fail(o, a);
            i++;
            /* The five names select rungs of the -V ladder, so a preset is a
             * quality choice like -V itself and is refused together with a tier,
             * a rate or a target PSNR for the same reason. */
            if (!strcmp(argv[i], "low"))
                o->v = KAC_PRESET_LOW;
            else if (!strcmp(argv[i], "medium"))
                o->v = KAC_PRESET_MEDIUM;
            else if (!strcmp(argv[i], "standard"))
                o->v = KAC_PRESET_STD;
            else if (!strcmp(argv[i], "extreme"))
                o->v = KAC_PRESET_EXTREME;
            else if (!strcmp(argv[i], "insane"))
                o->v = KAC_PRESET_INSANE;
            else
                return parse_fail(o, a);
            o->have_v       = 1;
            o->preset_named = 1;
            o->seen |= KAC_SEEN_V;
        } else if (!strcmp(a, "--loop")) {
            const char *p, *end;
            uint32_t    s, e;
            if (i + 1 >= argc)
                return parse_fail(o, a);
            p = argv[++i];
            if (parse_u32_span(p, &end, &s) != 0 || *end != ',')
                return parse_fail(o, a);
            if (parse_u32_span(end + 1, &end, &e) != 0 || *end != '\0')
                return parse_fail(o, a);
            if (s >= e)
                return parse_fail(o, a);
            o->loop_start = s;
            o->loop_end   = e;
            o->seen |= KAC_SEEN_LOOP;
        } else if (!strcmp(a, "--stride")) {
            uint32_t v;
            if (i + 1 >= argc || parse_u32(argv[++i], &v))
                return parse_fail(o, a);
            if (v == 0u)
                return parse_error(o, "--stride must be positive");
            o->stride = v;
            o->seen |= KAC_SEEN_STRIDE;
        } else if (!strcmp(a, "--limit")) {
            if (i + 1 >= argc || parse_u32(argv[++i], &o->limit))
                return parse_fail(o, a);
            o->have_limit = 1;
            o->seen |= KAC_SEEN_LIMIT;
        } else if (!strcmp(a, "--loop-count")) {
            if (i + 1 >= argc || parse_u32(argv[++i], &o->loops))
                return parse_fail(o, a);
            o->have_loops = 1;
        } else if (a[0] == '-' && a[1])
            return parse_unknown(o, a);
        else if (!o->in)
            o->in = a;
        else if (!o->out)
            o->out = a;
        else
            return parse_error(o, "too many file names");
    }
    /* Reject mutually exclusive quality/rate settings instead of silently
     * dropping one. -b and -B are checked later, when their tier folds are
     * known. */
    if (o->have_v && (o->abr > 0.0 || o->abr_bps > 0.0 || o->psnr > 0.0))
        return parse_error(o, "use one of -V/--preset, --psnr and --abr/--abr-bps");
    if (o->psnr > 0.0 && (o->abr > 0.0 || o->abr_bps > 0.0))
        return parse_error(o, "--psnr and --abr/--abr-bps are different modes; use one");
    if (o->abr > 0.0 && o->abr_bps > 0.0)
        return parse_error(o, "--abr and --abr-bps are one setting; use one");
    /* Direction: encoding-only options are refused on decode. */
    if (o->decode && o->seen)
        return parse_error(o, "encoding options cannot be combined with --decode");
    if (!o->decode && o->have_loops)
        return parse_error(o, "--loop-count belongs to --decode");
    if (o->lowdelay) {
        /* A stream cannot carry loop state or an index trailer. */
        if (o->loop_end > o->loop_start || o->stride != 0u)
            return parse_error(o, "--lowdelay cannot carry a loop region or a seek index");
        if (o->lowdelay_tier_named && (opts_named_setting(o) || o->tier || o->brate > 0.0))
            return parse_error(o, "--lowdelay names its tier; a VBR setting, --tier or -b "
                                  "cannot be combined with it");
    }
    if (o->have_chunk_ms && !o->lowdelay)
        return parse_error(o, "--chunk-ms belongs to --lowdelay");
    if (o->tier && (opts_named_setting(o) || o->brate > 0.0))
        return parse_error(o, "--tier cannot be combined with a VBR setting or -b");
    /* Nothing selected: the encoder's default applies. */
    if (!o->have_v && o->abr <= 0.0 && o->abr_bps <= 0.0 && !o->tier && o->brate <= 0.0 &&
        o->psnr <= 0.0)
        o->def_v = 1;
    if (o->in && !o->out && derive_out(o) != 0)
        return parse_error(o, "cannot derive an output name; name one");
    if (!o->in || !o->out)
        return parse_error(o, "expected an input and an output file");
    return 0;
}

/* Encode report: level, counters, and for VBR the target and offset. */
static void report_encode(const kac_opts *o, const kac_t *g, uint32_t rate, unsigned ch,
                          int abr_held)
{
    summary_line(o, kac_get_tier(g), kac_get_vbr_mode(g) != KAC_VBR_OFF, kac_get_brate_achieved(g),
                 kac_get_total(g), rate, (int)ch);
    fprintf(stderr, "PSNR %.3f dB  clip %.4f %%  |pred|>32767 %.4f %%  w_at_limit %.4f %%\n",
            kac_get_psnr(g), 100.0 * kac_get_clip_rate(g), 100.0 * kac_get_pred_over_rate(g),
            100.0 * kac_get_w_limit_rate(g));
    if (kac_get_vbr_mode(g) == KAC_VBR_OFF)
        return;
    if (o->have_v || o->def_v)
        fprintf(stderr, "VBR -V %.3f  target %.3f dB  nominal T%u\n", opts_rung1000(o) / 1000.0,
                kac_get_vbr_target(g), kac_get_vbr_nominal_tier(g));
    else if (o->psnr > 0.0)
        fprintf(stderr, "VBR --psnr %.3f dB  target %.3f dB  nominal T%u\n", o->psnr,
                kac_get_vbr_target(g), kac_get_vbr_nominal_tier(g));
    else
        fprintf(stderr, "VBR --abr %.4g bit/channel*sample%s  nominal T%u\n", kac_get_vbr_target(g),
                abr_held ? " (held at the window's end)" : "", kac_get_vbr_nominal_tier(g));
    fprintf(stderr, "VBR achieved %.4f bit/sample  %.3f dB  offset %+.4f  %u data bytes\n",
            kac_get_brate_achieved(g), kac_get_psnr(g), kac_get_vbr_offset(g),
            kac_get_data_bytes(g));
}

/* Low-delay stream: 36-byte preamble followed by chunks, with no placeholder. */
static int do_lowdelay(const kac_opts *o, const kac_wav *w, uint32_t frames)
{
    kac_t        g;
    kac_progress prog;
    FILE        *f   = NULL;
    uint8_t     *buf = NULL, *flush_buf = NULL;
    uint8_t      head[KAC_HDR_SIZE + 2u * KAC_SNAP_SIZE];
    size_t       headlen = 0, chunk_bytes, flush_cap;
    uint32_t     off;
    unsigned     tier = 0, chunk_ms = o->have_chunk_ms ? o->chunk_ms : 5u;
    unsigned     chunk, form;
    int          quality, head_written = 0;
    int          lim_lo = KAC_TIER_MIN, lim_hi = KAC_TIER_MAX, lim_held = 0, abr_held = 0;
    int          named     = opts_named_setting(o);
    int          use_tier  = o->tier ? o->tier : (o->lowdelay_tier_named ? o->lowdelay_tier : 0);
    int          per_block = named || (!use_tier && o->brate <= 0.0 && o->brate_max > 0.0);
    int          rc        = 1;

    prog.on = 0;
    quality = o->have_quality ? o->quality : 7;
    if (o->q_named && o->fh_named)
        warning("-q and -f/-h both set the quality level; using %d", quality);
    if (o->m_named && o->mono_named)
        warning("-m and -a/--mono both given; the run is %s",
                w->channels == 1 ? "one channel"
                                 : (o->ms_mode == KAC_MS_LR ? "left/right" : "mid/side"));
    if (o->have_limit && o->limit == 0)
        warning("--limit 0 means no limit");
    else if (o->have_limit && o->limit > w->frames)
        warning("--limit %u is past the file's %u frames; the whole file is encoded", o->limit,
                w->frames);
    /* The form is honoured when asked for; otherwise the mode keeps the low
     * latency default of one form for the whole stream. */
    if (w->channels == 1 || o->mono)
        form = KAC_MS_LR;
    else if (o->m_named || o->ms_auto_named)
        form = (unsigned)o->ms_mode;
    else
        form = KAC_MS_MS;
    if (w->channels == 1 && o->ms_mode == KAC_MS_MS)
        warning("the encoder input is mono; -m f has no effect");

    kac_init(&g);
    if (o->silent)
        kac_set_report(&g, 0);
    if (kac_set_num_channels(&g, (int)w->channels) != 0 ||
        kac_set_in_samplerate(&g, (int)w->rate) != 0)
        goto bad;
    if (named) {
        if (o->have_v) {
            if (kac_set_vbr(&g, KAC_VBR_V, opts_rung1000(o) / 1000.0) != 0)
                goto bad;
        } else if (o->psnr > 0.0) {
            if (kac_set_vbr_target_psnr(&g, o->psnr) != KAC_OK)
                goto bad;
        } else {
            double goal = (o->abr > 0.0) ? o->abr * 1000.0 / ((double)w->rate * (double)w->channels)
                                         : o->abr_bps;

            if (kac_set_vbr(&g, KAC_VBR_ABR, goal) != 0)
                goto bad;
            abr_held = ((int32_t)(goal * 1000.0 + 0.5) !=
                        (int32_t)(kac_get_vbr_target(&g) * 1000.0 + 0.5));
        }
    } else if (use_tier) {
        if (kac_set_tier(&g, use_tier) != 0)
            goto bad;
    } else if (o->brate > 0.0) {
        if (kac_set_brate(&g, o->brate) != 0)
            goto bad;
    } else if (per_block) {
        if (kac_set_vbr(&g, KAC_VBR_V, opts_rung1000(o) / 1000.0) != 0)
            goto bad;
    } else {
        tier = (unsigned)o->lowdelay_tier;
        if (kac_set_tier(&g, (int)tier) != 0)
            goto bad;
    }
    if (kac_set_quality(&g, quality) != 0)
        goto bad;
    if (o->brute && kac_set_brute(&g, (unsigned)o->brute) != 0)
        goto bad;
    if (kac_set_ms_mode(&g, (int)form) != 0)
        goto bad;
    if (per_block && (o->brate > 0.0 || o->brate_max > 0.0)) {
        lim_lo = (o->brate > 0.0) ? kac_tier_ceil_brate_vbr(o->brate) : KAC_TIER_MIN;
        lim_hi = (o->brate_max > 0.0) ? kac_tier_for_brate_vbr(o->brate_max) : KAC_TIER_MAX;
        if (lim_lo > lim_hi) {
            lim_hi   = lim_lo;
            lim_held = 1;
        }
        if (kac_set_tier_limits(&g, lim_lo, lim_hi) != 0)
            goto bad;
    }
    if (kac_init_params(&g) != 0)
        goto bad;
    if (kac_set_stream(&g, 1) != 0 || kac_set_chunk_ms(&g, chunk_ms) != 0) {
        fprintf(stderr,
                "kac: --lowdelay cannot make a %u ms chunk at %u Hz with this tier choice\n",
                chunk_ms, w->rate);
        return 2;
    }
    chunk = kac_get_chunk_samples(&g);

    if (!per_block && o->brate_max > 0.0)
        warning("-B is ignored: the tier is fixed");
    if (lim_held)
        warning("-b is above -B; the window is one tier (T%d) and its floor is kept", lim_lo);
    if (abr_held)
        warning("the rate target is outside the ladder's window; held at %.4g bit/channel*sample",
                kac_get_vbr_target(&g));

    if (!o->silent) {
        char        desc[64], cfg[160], level[16];
        const char *form_str =
            (w->channels == 1)
                ? "mono"
                : (form == KAC_MS_LR
                       ? "left/right"
                       : (form == KAC_MS_MS ? "mid/side"
                                            : (g.form != KAC_FORM_NONE ? "form measured"
                                                                       : "form as it arrives")));

        if (o->brute)
            snprintf(level, sizeof level, "brute %d", o->brute);
        else
            snprintf(level, sizeof level, "-q %u", quality);
        if (kac_get_vbr_mode(&g) == KAC_VBR_OFF)
            snprintf(cfg, sizeof cfg, "T%u, %.4f bit/sample", kac_get_tier(&g), kac_get_brate(&g));
        else if (o->psnr > 0.0)
            snprintf(cfg, sizeof cfg, "--psnr %.3f dB target, nominal T%u", o->psnr,
                     kac_get_vbr_nominal_tier(&g));
        else if (o->abr > 0.0 || o->abr_bps > 0.0)
            snprintf(cfg, sizeof cfg, "--abr %.4g bit/channel*sample%s, nominal T%u",
                     kac_get_vbr_target(&g), abr_held ? " (held at the window's end)" : "",
                     kac_get_vbr_nominal_tier(&g));
        else
            snprintf(cfg, sizeof cfg, "-V %.3f, %.3f dB quality target, nominal T%u",
                     opts_rung1000(o) / 1000.0, kac_get_vbr_target(&g),
                     kac_get_vbr_nominal_tier(&g));
        stream_desc(desc, sizeof desc, (unsigned)w->rate, w->channels);
        fprintf(stderr, "%s\n", kac_version());
        fprintf(stderr, "Encoding \"%s\" to \"%s\"\n", o->in, o->out);
        if (kac_get_vbr_mode(&g) == KAC_VBR_OFF)
            fprintf(stderr, "Encoding as %s, T%u, %s, chunk %u samples, %s", desc, kac_get_tier(&g),
                    form_str, chunk, level);
        else
            fprintf(stderr, "Encoding as %s, %s, %s, chunk %u samples, %s", desc, level, form_str,
                    chunk, cfg);
        fputc('\n', stderr);
        fflush(stderr);
    }
    progress_init(&prog, frames, (unsigned)w->rate, !o->silent);

    chunk_bytes = kac_encode_bufsize(&g, KAC_CHUNK_FRAMES);
    flush_cap   = kac_encode_bufsize(&g, frames);
    buf         = (uint8_t *)malloc(chunk_bytes);
    flush_buf   = (uint8_t *)malloc(flush_cap + 64u);
    f           = fopen(o->out, "wb");
    if (!buf || !flush_buf || !f) {
        fprintf(stderr, "kac: out of memory or cannot write %s\n", o->out);
        goto done;
    }
    for (off = 0; off < frames; off += KAC_CHUNK_FRAMES) {
        uint32_t nitems = frames - off;
        size_t   got    = 0;

        if (nitems > KAC_CHUNK_FRAMES)
            nitems = KAC_CHUNK_FRAMES;
        if (kac_encode_buffer_interleaved(&g, w->pcm + (size_t)off * w->channels, nitems, buf,
                                          &got) < 0) {
            fprintf(stderr, "kac: encode failed\n");
            goto done;
        }
        if (got > 0u) {
            /* The preamble states chunk 0's tier and form, so it exists only
             * once a chunk has been coded; it goes out before the first body
             * byte, whenever that byte appears. */
            if (!head_written) {
                if (kac_write_stream_preamble(&g, head, &headlen) != 0) {
                    fprintf(stderr, "kac: cannot write the stream preamble\n");
                    goto done;
                }
                if (fwrite(head, 1, headlen, f) != headlen) {
                    fprintf(stderr, "kac: cannot write %s\n", o->out);
                    goto done;
                }
                head_written = 1;
            }
            if (fwrite(buf, 1, got, f) != got) {
                fprintf(stderr, "kac: cannot write %s\n", o->out);
                goto done;
            }
        }
        progress_set(&prog, off + nitems);
    }
    {
        size_t got = 0;

        if (kac_encode_flush(&g, flush_buf, &got) < 0) {
            fprintf(stderr, "kac: flush failed\n");
            goto done;
        }
        if (got > 0u) {
            if (!head_written) {
                if (kac_write_stream_preamble(&g, head, &headlen) != 0) {
                    fprintf(stderr, "kac: cannot write the stream preamble\n");
                    goto done;
                }
                if (fwrite(head, 1, headlen, f) != headlen) {
                    fprintf(stderr, "kac: cannot write %s\n", o->out);
                    goto done;
                }
                head_written = 1;
            }
            if (fwrite(flush_buf, 1, got, f) != got) {
                fprintf(stderr, "kac: cannot write %s\n", o->out);
                goto done;
            }
        }
    }
    fclose(f);
    f = NULL;
    progress_finish(&prog);
    if (!o->silent)
        report_encode(o, &g, (uint32_t)w->rate, (unsigned)w->channels, 0);
    rc = 0;
done:
    progress_abandon(&prog);
    if (f)
        fclose(f);
    free(buf);
    free(flush_buf);
    kac_close(&g);
    return rc;
bad:
    fprintf(stderr, "kac: invalid configuration\n");
    return 2;
}

static int do_encode(const kac_opts *o)
{
    kac_wav      w;
    kac_t        g;
    kac_progress prog;
    FILE        *f   = NULL;
    uint8_t     *buf = NULL, *index_store = NULL, *flush_buf = NULL;
    uint32_t    *index_off = NULL;
    uint8_t      head[KAC_HDR_SIZE + 2u * KAC_SNAP_SIZE];
    size_t       headlen = 0, flush_cap;
    uint32_t     frames, off, entries = 0;
    size_t       chunk_bytes;
    int          rc       = 1;
    int          abr_held = 0;
    int          lim_lo = KAC_TIER_MIN, lim_hi = KAC_TIER_MAX;
    int          lim_held = 0; /* the two ends crossed and the window is one tier */

    prog.on = 0;
    if (wav_read(o->in, &w) != 0) {
        fprintf(stderr, "kac: cannot read %s (needs 16 bit PCM, 1 or 2 channels)\n", o->in);
        return 1;
    }
    frames = w.frames;
    if (w.rate == 0) {
        fprintf(stderr, "kac: %s states a sample rate of 0\n", o->in);
        goto done;
    }
    if (o->limit != 0 && o->limit < frames)
        frames = o->limit;
    if (o->mono && w.channels == 2) {
        /* One channel out of two: their average, the mid the coded form
         * already builds. The read is in front of the write, so the buffer
         * converts in place. */
        uint32_t i;

        for (i = 0; i < frames; i++)
            w.pcm[i] = (int16_t)(((int32_t)w.pcm[2u * i] + (int32_t)w.pcm[2u * i + 1u]) >> 1);
        w.channels = 1;
    }
    /* Failures from here go through `done`, which releases what has been taken.
     * The failed wav_read() above cannot come here: it leaves `w` untouched and
     * so must not be freed. */
    if (frames == 0) {
        fprintf(stderr, "kac: no samples to encode\n");
        goto done;
    }
    /* The loop region indexes the samples this run encodes, so its end is held
     * to that count -- after --limit, which is what the file will state. */
    if (o->loop_end > frames) {
        fprintf(stderr, "kac: loop region %u,%u exceeds the %u frames to encode\n", o->loop_start,
                o->loop_end, frames);
        rc = 2;
        goto done;
    }
    if (o->lowdelay) {
        /* A different emission path: the output is a stream, with no header to
         * patch and no per-block choice to make. */
        rc = do_lowdelay(o, &w, frames);
        goto done;
    }

    kac_init(&g);
    if (o->silent)
        kac_set_report(&g, 0);
    kac_set_num_channels(&g, (int)w.channels);
    kac_set_in_samplerate(&g, (int)w.rate);
    if (o->have_v || o->def_v)
        kac_set_vbr(&g, KAC_VBR_V, opts_rung1000(o) / 1000.0);
    else if (o->abr > 0.0 || o->abr_bps > 0.0) {
        /* --abr takes kbit/s and converts with this file's rate and channels;
         * --abr-bps is already bit/sample. Goals outside the rate window are
         * held, so compare the stored rounded value and report that. */
        double goal =
            (o->abr > 0.0) ? o->abr * 1000.0 / ((double)w.rate * (double)w.channels) : o->abr_bps;

        kac_set_vbr(&g, KAC_VBR_ABR, goal);
        abr_held =
            ((int32_t)(goal * 1000.0 + 0.5) != (int32_t)(kac_get_vbr_target(&g) * 1000.0 + 0.5));
    }
    if (o->tier)
        kac_set_tier(&g, o->tier);
    /* -b names a rate; a VBR setting decides how it reads and leaves it to the
     * window below. Only -b alone pins a tier. */
    if (o->brate > 0.0 && !opts_named_setting(o))
        kac_set_brate(&g, o->brate);
    if (o->have_quality)
        kac_set_quality(&g, o->quality);
    if (o->brute)
        kac_set_brute(&g, (unsigned)o->brute);
    if (o->ms_mode != KAC_MS_AUTO)
        kac_set_ms_mode(&g, o->ms_mode);
    if (o->loop_end > o->loop_start && kac_set_loop(&g, o->loop_start, o->loop_end) != 0) {
        fprintf(stderr, "kac: bad loop region\n");
        rc = 2;
        goto done;
    }
    if (o->psnr > 0.0 && kac_set_vbr_target_psnr(&g, o->psnr) != KAC_OK) {
        fprintf(stderr, "kac: cannot set that quality target\n");
        rc = 2;
        goto done;
    }
    if (kac_get_vbr_mode(&g) != KAC_VBR_OFF && (o->brate > 0.0 || o->brate_max > 0.0)) {
        /* Rate bounds fold to a tier window over T1..T7. If they cross, keep
         * the lower bound and make the window one tier. Set after kac_set_vbr(),
         * which opens the window. */
        lim_lo = (o->brate > 0.0) ? kac_tier_ceil_brate_vbr(o->brate) : KAC_TIER_MIN;
        lim_hi = (o->brate_max > 0.0) ? kac_tier_for_brate_vbr(o->brate_max) : KAC_TIER_MAX;
        if (lim_lo > lim_hi) {
            lim_hi   = lim_lo;
            lim_held = 1;
        }
        /* Set last: kac_set_vbr() would overwrite it. */
        kac_set_tier_limits(&g, lim_lo, lim_hi);
    }
    if (o->stride != 0) {
        unsigned blk = kac_block_for_rate(w.rate);

        if (o->stride % blk != 0u) {
            fprintf(stderr, "kac: --stride must be a multiple of the %u-sample block at %u Hz\n",
                    blk, w.rate);
            rc = 2;
            goto done;
        }
        entries     = (frames - 1u) / o->stride + 1u;
        index_store = (uint8_t *)malloc((size_t)entries * w.channels * KAC_SNAP_SIZE);
        if (!index_store) {
            fprintf(stderr, "kac: out of memory for the seek index\n");
            goto done;
        }
        if (kac_set_index(&g, o->stride, index_store,
                          (size_t)entries * w.channels * KAC_SNAP_SIZE) != 0) {
            fprintf(stderr, "kac: bad stride\n");
            goto done;
        }
        /* Offsets are recorded per block: the tier byte fixes each block's
         * length. */
        index_off = (uint32_t *)malloc((size_t)entries * sizeof(uint32_t));
        if (!index_off || kac_set_index_offsets(&g, index_off, entries) != 0) {
            fprintf(stderr, "kac: out of memory\n");
            goto done;
        }
    }
    if (kac_init_params(&g) != 0) {
        fprintf(stderr, "kac: invalid configuration\n");
        rc = 2;
        goto done;
    }

    /* Legal configurations that do something other than what they look like.
     * The run goes ahead; the lines go out whether or not -S is on. */
    {
        int end_tier = 0;

        if (o->q_named && o->fh_named)
            warning("-q and -f/-h both set the quality level; using %d", o->quality);
        if (o->v_named && o->preset_named)
            warning("--preset and -V both set the rung; using %.3f", opts_rung1000(o) / 1000.0);
        if (o->m_named && o->mono_named)
            warning("-m and -a/--mono both given; the run is %s",
                    o->mono
                        ? "one channel"
                        : (o->ms_mode == KAC_MS_LR
                               ? "left/right"
                               : (o->ms_mode == KAC_MS_MS ? "mid/side" : "measured per block")));
        if (kac_get_vbr_mode(&g) == KAC_VBR_OFF) {
            if (o->brate > 0.0 && !opts_named_setting(o) &&
                rate_outside_window(o->brate, &end_tier, KAC_TIER_FIXED_MIN, KAC_TIER_FIXED_MAX) !=
                    0)
                warning("-b %.4g is outside the tier range; the run sits on T%d", o->brate,
                        end_tier);
            if (o->brate_max > 0.0)
                warning("-B %.4g is ignored: the tier is fixed", o->brate_max);
        } else {
            if (o->brate > 0.0 &&
                rate_outside_window(o->brate, &end_tier, KAC_TIER_MIN, KAC_TIER_MAX) != 0)
                warning("-b %.4g is outside the per-block window; the floor is held at T%d",
                        o->brate, end_tier);
            if (o->brate_max > 0.0 &&
                rate_outside_window(o->brate_max, &end_tier, KAC_TIER_MIN, KAC_TIER_MAX) != 0)
                warning("-B %.4g is outside the per-block window; the ceiling is held at T%d",
                        o->brate_max, end_tier);
            if (lim_held)
                warning("-b %.4g is above -B %.4g; the window is one tier (T%d) and its floor "
                        "is kept",
                        o->brate, o->brate_max, lim_lo);
        }
        if (abr_held)
            warning("the rate target is outside the ladder's window; held at %.4g "
                    "bit/channel*sample",
                    kac_get_vbr_target(&g));
        if (o->psnr > 0.0 && (o->psnr < 33.75 || o->psnr > 63.00))
            warning("--psnr %.3f dB is outside the ladder's 33.75..63.00 dB range", o->psnr);
        if (w.channels == 1 && o->ms_mode == KAC_MS_MS)
            warning("the encoder input is mono; -m f has no effect");
        if (o->have_limit && o->limit == 0)
            warning("--limit 0 means no limit");
        else if (o->have_limit && o->limit > w.frames)
            warning("--limit %u is past the file's %u frames; the whole file is encoded", o->limit,
                    w.frames);
    }

    if (!o->silent) {
        /* Read the configuration from the handle, since -b/-B alone leave the
         * default rung in place. */
        char   cfg[160], desc[64], level[16];
        size_t used;
        if (kac_get_vbr_mode(&g) == KAC_VBR_OFF)
            snprintf(cfg, sizeof cfg, "T%u, %.4f bit/sample", kac_get_tier(&g), kac_get_brate(&g));
        else if (o->abr_bps > 0.0)
            snprintf(cfg, sizeof cfg, "--abr-bps %.4g bit/channel*sample%s, quality floats",
                     kac_get_vbr_target(&g), abr_held ? " (held at the window's end)" : "");
        else if (o->abr > 0.0)
            snprintf(cfg, sizeof cfg, "--abr %.4g kbit/s = %.4g bit/channel*sample%s", o->abr,
                     kac_get_vbr_target(&g), abr_held ? " (held at the window's end)" : "");
        else if (o->psnr > 0.0)
            snprintf(cfg, sizeof cfg, "--psnr %.3f dB target, nominal T%u", o->psnr,
                     kac_get_vbr_nominal_tier(&g));
        else
            snprintf(cfg, sizeof cfg, "-V %.3f, %.3f dB quality target", opts_rung1000(o) / 1000.0,
                     kac_get_vbr_target(&g));
        used = strlen(cfg);
        /* Report a one-tier window or an ignored -B. */
        if (kac_get_vbr_mode(&g) != KAC_VBR_OFF && (o->brate > 0.0 || o->brate_max > 0.0))
            snprintf(cfg + used, sizeof cfg - used, ", tiers T%d..T%d%s", lim_lo, lim_hi,
                     lim_held ? " (held: -B is under -b)" : "");
        else if (o->brate_max > 0.0)
            snprintf(cfg + used, sizeof cfg - used, " (-B ignored: the tier is fixed)");
        if (o->brute)
            snprintf(level, sizeof level, "brute %d", o->brute);
        else
            snprintf(level, sizeof level, "-q %u", kac_get_quality(&g));
        stream_desc(desc, sizeof desc, w.rate, w.channels);
        fprintf(stderr, "%s\n", kac_version());
        fprintf(stderr, "Encoding \"%s\" to \"%s\"\n", o->in, o->out);
        if (w.channels == 1)
            fprintf(stderr, "Encoding as %s, %s, %s\n", desc, level, cfg);
        else {
            const char *form =
                (o->ms_mode == KAC_MS_LR) ? "left/right"
                : (o->ms_mode == KAC_MS_MS)
                    ? "mid/side"
                    : (g.form != KAC_FORM_NONE ? "form measured" : "form as it arrives");

            fprintf(stderr, "Encoding as %s, %s, %s, %s\n", desc, level, form, cfg);
        }
        fflush(stderr);
    }
    progress_init(&prog, frames, w.rate, !o->silent);

    f           = fopen(o->out, "wb");
    chunk_bytes = kac_encode_bufsize(&g, KAC_CHUNK_FRAMES);
    /* The flush includes the seek index, whose size depends on the total. */
    flush_cap = kac_encode_bufsize(&g, frames);
    buf       = (uint8_t *)malloc(chunk_bytes);
    flush_buf = (uint8_t *)malloc(flush_cap + 64u);
    if (!f || !buf || !flush_buf) {
        fprintf(stderr, "kac: out of memory or cannot write %s\n", o->out);
        goto done;
    }
    for (off = 0; off < frames; off += KAC_CHUNK_FRAMES) {
        uint32_t n   = frames - off;
        size_t   got = 0;
        if (n > KAC_CHUNK_FRAMES)
            n = KAC_CHUNK_FRAMES;
        if (kac_encode_buffer_interleaved(&g, w.pcm + (size_t)off * w.channels, n, buf, &got) < 0 ||
            fwrite(buf, 1, got, f) != got) {
            fprintf(stderr, "kac: encode failed\n");
            goto done;
        }
        progress_set(&prog, off + n);
    }
    {
        size_t got = 0;
        if (kac_encode_flush(&g, flush_buf, &got) < 0 || fwrite(flush_buf, 1, got, f) != got) {
            fprintf(stderr, "kac: flush failed\n");
            goto done;
        }
    }
    if (kac_write_preamble(&g, head, &headlen) != 0 || fseek(f, 0, SEEK_SET) != 0 ||
        fwrite(head, 1, headlen, f) != headlen) {
        fprintf(stderr, "kac: cannot patch the header in %s\n", o->out);
        goto done;
    }
    fclose(f);
    f = NULL;
    progress_finish(&prog);
    if (!o->silent)
        report_encode(o, &g, (uint32_t)w.rate, (unsigned)w.channels, abr_held);
    rc = 0;
done:
    progress_abandon(&prog);
    if (f)
        fclose(f);
    free(buf);
    free(flush_buf);
    free(index_off);
    free(index_store);
    wav_free(&w);
    return rc;
}

static uint32_t le32(const unsigned char *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Decode a stream: every chunk but the last holds the sample count the preamble
 * states, and its slice count follows from that and its own tier byte. The last
 * chunk is whatever is left. Returns a process exit code. */
static int do_stream_decode(const kac_opts *o, kacdec_t *restrict d, const uint8_t *image,
                            size_t len)
{
    kac_progress prog;
    FILE        *f = NULL;
    int16_t      pcm[KAC_CHUNK_FRAMES * KAC_MAX_CHANNELS];
    size_t      *off = NULL, *blen = NULL;
    size_t       pos, hlen;
    unsigned     nk = 0, cap = 0, i, ch;
    uint32_t     total = 0, done = 0, rate, chunk_samples;
    int          rc = 1;

    prog.on = 0;
    if (len < (size_t)KAC_HDR_SIZE) {
        fprintf(stderr, "kac: %s is too short for a stream preamble\n", o->in);
        return 1;
    }
    hlen = KAC_HDR_SIZE + ((image[5] & KAC_FLAG_INIT) ? (size_t)image[6] * KAC_SNAP_SIZE : 0u);
    if (hlen > len) {
        fprintf(stderr, "kac: %s is too short for its preamble\n", o->in);
        return 1;
    }
    {
        int orc = kacdec_stream_open(d, image, hlen);

        if (orc != KAC_OK) {
            fprintf(stderr, "kac: %s rejected (error %d)\n", o->in, orc);
            return 1;
        }
    }
    if (o->loops > 0) {
        fprintf(stderr, "kac: a stream has no loop region\n");
        return 1;
    }
    /* The preamble states the chunk's sample count; the stream front end has
     * already held it to 1..KAC_CHUNK_SAMPLES. */
    chunk_samples = le32(image + 28);
    ch            = kacdec_channels(d);
    rate          = kacdec_sample_rate(d);

    /* The scan: one tier byte, then the slices. A chunk is full while the bytes
     * are there for the full count; what is left after the last full one is the
     * sender's final (short) chunk. */
    for (pos = hlen; pos < len;) {
        size_t   avail = len - pos;
        unsigned t     = image[pos] & KAC_TIER_MASK;
        unsigned n, full, nsl;
        size_t   bytes;

        if (t < KAC_TIER_MIN || t > KAC_TIER_MAX) {
            fprintf(stderr, "kac: chunk at byte %zu carries tier %u\n", pos, t);
            rc = 2;
            goto done;
        }
        n = (unsigned)kac_tiers[t - 1u].n;
        if (chunk_samples % n != 0u) {
            fprintf(stderr, "kac: a %u-sample chunk is not a whole number of T%u slices (N = %u)\n",
                    chunk_samples, t, n);
            rc = 2;
            goto done;
        }
        full = chunk_samples / n;
        if (avail >= 1u + (size_t)full * KAC_SLICE_SIZE * ch) {
            nsl   = full;
            bytes = 1u + (size_t)nsl * KAC_SLICE_SIZE * ch;
        } else {
            if ((avail - 1u) % ((size_t)KAC_SLICE_SIZE * ch) != 0u) {
                fprintf(stderr, "kac: the last chunk at byte %zu is not whole slices\n", pos);
                rc = 2;
                goto done;
            }
            nsl   = (unsigned)((avail - 1u) / ((size_t)KAC_SLICE_SIZE * ch));
            bytes = avail;
            if (nsl == 0u || nsl > full) {
                fprintf(stderr, "kac: the last chunk at byte %zu has %u slices\n", pos, nsl);
                rc = 2;
                goto done;
            }
        }
        if ((uint64_t)total + (uint64_t)nsl * n > (uint64_t)KAC_MAX_TOTAL) {
            fprintf(stderr, "kac: the stream is longer than the format allows\n");
            rc = 2;
            goto done;
        }
        total += nsl * n;
        if (nk == cap) {
            unsigned ncap = cap ? cap * 2u : 256u;
            size_t  *no   = (size_t *)realloc(off, (size_t)ncap * sizeof *off);

            if (!no) {
                fprintf(stderr, "kac: out of memory\n");
                goto done;
            }
            off = no;
            {
                size_t *nb = (size_t *)realloc(blen, (size_t)ncap * sizeof *blen);

                if (!nb) {
                    fprintf(stderr, "kac: out of memory\n");
                    goto done;
                }
                blen = nb;
            }
            cap = ncap;
        }
        off[nk]  = pos;
        blen[nk] = bytes;
        nk++;
        pos += bytes;
    }
    if (nk == 0) {
        fprintf(stderr, "kac: %s carries no chunks\n", o->in);
        goto done;
    }

    if (!o->silent) {
        char desc[64];
        stream_desc(desc, sizeof desc, rate, ch);
        fprintf(stderr, "%s\n", kac_version());
        fprintf(stderr, "Decoding \"%s\" to \"%s\"\n", o->in, o->out);
        fprintf(stderr, "Decoding as %s, chunked stream\n", desc);
        fflush(stderr);
    }
    progress_init(&prog, total, rate, !o->silent);

    f = fopen(o->out, "wb");
    if (!f || wav_write_header(f, total, ch, rate) != 0) {
        fprintf(stderr, "kac: cannot write %s\n", o->out);
        goto done;
    }
    for (i = 0; i < nk; i++) {
        int32_t got;

        if (kacdec_stream_feed(d, image + off[i], blen[i]) != KAC_OK) {
            fprintf(stderr, "kac: chunk %u rejected\n", i);
            goto done;
        }
        while ((got = kacdec_read(d, pcm, KAC_CHUNK_FRAMES)) > 0) {
            if (wav_write_frames(f, pcm, (uint32_t)got, ch) != 0) {
                fprintf(stderr, "kac: cannot write %s\n", o->out);
                goto done;
            }
            done += (uint32_t)got;
            progress_set(&prog, done);
        }
    }
    progress_finish(&prog);
    if (done != total) {
        fprintf(stderr, "kac: stream decoded %u of %u frames\n", done, total);
        goto done;
    }
    if (!o->silent) {
        double brate = (double)(len - hlen) * 8.0 / ((double)done * (double)ch);
        summary_line(o, kacdec_tier(d), 0, brate, done, rate, ch);
    }
    rc = 0;
done:
    progress_abandon(&prog);
    if (f)
        fclose(f);
    free(off);
    free(blen);
    return rc;
}

static int do_decode(const kac_opts *o)
{
    FILE        *f;
    long         fsz;
    uint8_t     *image = NULL;
    kacdec_t     d;
    int16_t      chunk[KAC_CHUNK_FRAMES * KAC_MAX_CHANNELS];
    uint32_t     total, done = 0, ch, rate;
    kac_progress prog;
    int          rc = 1;

    prog.on = 0;
    if (o->have_loops && o->loops == 0)
        warning("--loop-count 0 means the file is decoded once");
    f = fopen(o->in, "rb");
    if (!f) {
        fprintf(stderr, "kac: cannot open %s\n", o->in);
        return 1;
    }
    if (fseek(f, 0, SEEK_END) || (fsz = ftell(f)) < 0 || fseek(f, 0, SEEK_SET)) {
        fclose(f);
        return 1;
    }
    image = (uint8_t *)malloc((size_t)fsz);
    if (!image || fread(image, 1, (size_t)fsz, f) != (size_t)fsz) {
        fprintf(stderr, "kac: cannot read %s\n", o->in);
        goto done;
    }
    fclose(f);
    f = NULL;

    kacdec_init(&d);
    /* A stream states no length: TotalSamples and DataBytes are zero, and its
     * chunk's sample count is in IndexStride. */
    if (fsz >= (long)KAC_HDR_SIZE && le32(image + 12) == 0u && le32(image + 24) == 0u) {
        rc = do_stream_decode(o, &d, image, (size_t)fsz);
        goto done;
    }
    rc = kacdec_open(&d, image, (size_t)fsz);
    if (rc != KAC_OK) {
        fprintf(stderr, "kac: %s rejected (error %d)\n", o->in, rc);
        rc = 1;
        goto done;
    }
    ch    = kacdec_channels(&d);
    rate  = kacdec_sample_rate(&d);
    total = kacdec_total(&d);
    if (o->loops > 0) {
        /* The loop region played K times, in 64 bit: a WAV states its frame
         * count in 32, so a request whose total does not fit it is one this
         * tool cannot carry out, and it says so rather than wrapping. */
        uint64_t span, want;

        if (!(d.flags & KAC_FLAG_LOOP)) {
            fprintf(stderr, "kac: file has no loop region\n");
            goto done;
        }
        span = (uint64_t)(d.loop_end - d.loop_start);
        want = (uint64_t)d.loop_start + span * (uint64_t)o->loops;
        /* The WAV states its frame count and its byte count in 32 bits. */
        if (want > (uint64_t)0xffffffffu ||
            want * (uint64_t)ch * 2u + 44u > (uint64_t)0xffffffffu) {
            fprintf(stderr, "kac: --loop-count %u asks for more samples than a WAV states\n",
                    o->loops);
            rc = 1;
            goto done;
        }
        total = (uint32_t)want;
    }

    if (!o->silent) {
        char desc[64];
        stream_desc(desc, sizeof desc, rate, ch);
        fprintf(stderr, "%s\n", kac_version());
        fprintf(stderr, "Decoding \"%s\" to \"%s\"\n", o->in, o->out);
        fprintf(stderr, "Decoding as %s%s\n", desc, o->loops > 0 ? ", looping" : "");
        fflush(stderr);
    }
    progress_init(&prog, total, rate, !o->silent);

    f = fopen(o->out, "wb");
    if (!f || wav_write_header(f, total, ch, rate) != 0) {
        fprintf(stderr, "kac: cannot write %s\n", o->out);
        goto done;
    }
    while (done < total) {
        uint32_t want = total - done;
        int32_t  got;
        if (want > KAC_CHUNK_FRAMES)
            want = KAC_CHUNK_FRAMES;
        if (o->loops > 0 && done + want > d.loop_start) {
            if (done < d.loop_start) {
                got = kacdec_read(&d, chunk, d.loop_start - done);
            } else {
                kacdec_set_loop(&d, 1);
                got = kacdec_read(&d, chunk, want);
            }
        } else {
            got = kacdec_read(&d, chunk, want);
        }
        if (got < 0) {
            fprintf(stderr, "kac: decode failed (%d)\n", got);
            goto done;
        }
        if (got == 0)
            break;
        if (wav_write_frames(f, chunk, (uint32_t)got, ch) != 0) {
            fprintf(stderr, "kac: cannot write %s\n", o->out);
            goto done;
        }
        done += (uint32_t)got;
        progress_set(&prog, done);
    }
    progress_finish(&prog);
    if (!o->silent) {
        double brate = (double)d.data_bytes * 8.0 / ((double)d.total * (double)ch);
        summary_line(o, kacdec_tier(&d), 0, brate, done, rate, ch);
    }
    rc = 0;
done:
    progress_abandon(&prog);
    if (f)
        fclose(f);
    kacdec_close(&d);
    free(image);
    return rc;
}

int main(int argc, char **argv)
{
    kac_opts o;

    memset(&o, 0, sizeof o);
    if (parse_opts(argc, argv, &o) != 0) {
        if (o.err[0])
            fprintf(stderr, "kac: %s\n", o.err);
        fprintf(stderr, "Try 'kac --help'.\n");
        return 2;
    }
    if (!strcmp(o.in, "-") || !strcmp(o.out, "-")) {
        fprintf(stderr, "kac: \"-\" (stdin/stdout) is not supported; name a file\n");
        return 2;
    }
    if (!strcmp(o.in, o.out)) {
        fprintf(stderr, "kac: input and output are the same file\n");
        return 2;
    }
    return o.decode ? do_decode(&o) : do_encode(&o);
}
