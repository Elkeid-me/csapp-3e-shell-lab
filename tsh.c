/**
 * tsh - A tiny shell program with job control
 *
 *
 */
#include <assert.h>
#include <bits/types/sigset_t.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

/* Misc manifest constants */
#define MAXLINE 1024   /* max line size */
#define MAXARGS 128    /* max args on a command line */
#define MAXJOBS 16     /* max jobs at any point in time */
#define MAXJID 1 << 16 /* max job ID */

/* Job states */
#define UNDEF 0 /* undefined */
#define FG 1    /* running in foreground */
#define BG 2    /* running in background */
#define ST 3    /* stopped */

const char command_name[3][5] = {"kill", "bg", "fg"};

/*
 * Jobs states: FG (foreground), BG (background), ST (stopped)
 * Job state transitions and enabling actions:
 *     FG -> ST  : ctrl-z
 *     ST -> FG  : fg command
 *     ST -> BG  : bg command
 *     BG -> FG  : fg command
 * At most 1 job can be in the FG state.
 */

/* Parsing states */
#define ST_NORMAL 0x0  /* next token is an argument */
#define ST_INFILE 0x1  /* next token is the input file */
#define ST_OUTFILE 0x2 /* next token is the output file */

/* Global variables */
extern char **environ;   /* defined in libc */
char prompt[] = "tsh> "; /* command line prompt (DO NOT CHANGE) */
int verbose = 0;         /* if true, print additional output */
int nextjid = 1;         /* next job ID to allocate */
char sbuf[MAXLINE];      /* for composing sprintf messages */

struct job_t
{                          /* The job struct */
    pid_t pid;             /* job PID */
    int jid;               /* job ID [1, 2, ...] */
    int state;             /* UNDEF, BG, FG, or ST */
    char cmdline[MAXLINE]; /* command line */
};

struct job_t job_list[MAXJOBS]; /* The job list */

struct cmdline_tokens
{
    int argc;            /* Number of arguments */
    char *argv[MAXARGS]; /* The arguments list */
    char *infile;        /* The input file */
    char *outfile;       /* The output file */
    enum builtins_t
    { /* Indicates if argv[0] is a builtin command */
      BUILTIN_NONE,
      BUILTIN_QUIT,
      BUILTIN_JOBS,
      BUILTIN_BG,
      BUILTIN_FG,
      BUILTIN_KILL,
      BUILTIN_NOHUP
    } builtins;
};

/* End global variables */

#define MACRO_TO_VAL(x) #x
#define VAL_TO_STR(x) MACRO_TO_VAL(x)

/**
 * 处理系统调用错误的宏. 当系统调用返回 -1 时, 打印函数名称、行号以及错误原因,
 * 之后退出.
 */
#define handle_system_call(func)                                               \
    if ((func) < 0)                                                            \
        unix_error("Error when calling " #func                                 \
                   " in line " VAL_TO_STR(__LINE__));

/* Function prototypes */
void eval(const char *const cmdline);

int is_valid_pid(const int pid);
int is_valid_jid(const int jid);
void builtin_jobs(const char *const out_file);

void run_outside_command(char *const *argv, const char *const in_file,
                         const char *const out_file, const char *const cmd_line,
                         const int is_bg);

void wait_for_fg_exit();

void builtin_kill_bg_fg_trait(const char *const arg,
                              const enum builtins_t command_type);
void builtin_kill_bg_fg_impl(const enum builtins_t command_type, const int pid);

void sigchld_handler(int sig);
void sigtstp_handler(int sig);
void sigint_handler(int sig);

/* Here are helper routines that we've provided for you */
int parseline(const char *cmdline, struct cmdline_tokens *tok);
void sigquit_handler(int sig);

void clearjob(struct job_t *job);
void initjobs(struct job_t *job_list);
int maxjid(struct job_t *job_list);
int addjob(struct job_t *job_list, pid_t pid, int state, const char *cmdline);
int deletejob(struct job_t *job_list, pid_t pid);
pid_t fgpid(struct job_t *job_list);
struct job_t *getjobpid(struct job_t *job_list, pid_t pid);
struct job_t *getjobjid(struct job_t *job_list, int jid);
int pid2jid(pid_t pid);
void listjobs(struct job_t *job_list, int output_fd);

void usage(void);
void unix_error(char *msg);
void app_error(char *msg);
ssize_t sio_puts(const char s[]);
ssize_t sio_putl(long v);
ssize_t sio_put(const char *fmt, ...);
void sio_error(char s[]);

typedef void handler_t(int);
handler_t *Signal(int signum, handler_t *handler);

/*
 * main - The shell's main routine
 */
int main(int argc, char **argv)
{
    char c;
    char cmdline[MAXLINE]; /* cmdline for fgets */
    int emit_prompt = 1;   /* emit prompt (default) */

    /* Redirect stderr to stdout (so that driver will get all output
     * on the pipe connected to stdout) */
    /*
     * 原本这里是 dup2(1, 2), 没有检查系统调用的返回值, 且使用了 magic number,
     * 这是一种极不负责的行为.
     *
     * 建议给 CMU 扣分.
     */
    handle_system_call(dup2(STDOUT_FILENO, STDERR_FILENO));

    /* Parse the command line */
    while ((c = getopt(argc, argv, "hvp")) != EOF)
    {
        switch (c)
        {
        case 'h': /* print help message */
            usage();
            break;
        case 'v': /* emit additional diagnostic info */
            verbose = 1;
            break;
        case 'p':            /* don't print a prompt */
            emit_prompt = 0; /* handy for automatic testing */
            break;
        default:
            usage();
        }
    }

    /* Install the signal handlers */

    /* These are the ones you will need to implement */
    Signal(SIGINT, sigint_handler);   /* ctrl-c */
    Signal(SIGTSTP, sigtstp_handler); /* ctrl-z */
    Signal(SIGCHLD, sigchld_handler); /* Terminated or stopped child */
    Signal(SIGTTIN, SIG_IGN);
    Signal(SIGTTOU, SIG_IGN);

    /* This one provides a clean way to kill the shell */
    Signal(SIGQUIT, sigquit_handler);

    /* Initialize the job list */
    initjobs(job_list);

    /* Execute the shell's read/eval loop */
    while (1)
    {

        if (emit_prompt)
        {
            printf("%s", prompt);
            fflush(stdout);
        }
        if ((fgets(cmdline, MAXLINE, stdin) == NULL) && ferror(stdin))
            app_error("fgets error");
        if (feof(stdin))
        {
            /* End of file (ctrl-d) */
            printf("\n");
            fflush(stdout);
            fflush(stderr);
            exit(0);
        }

        /* Remove the trailing newline */
        cmdline[strlen(cmdline) - 1] = '\0';

        /* Evaluate the command line */
        eval(cmdline);

        fflush(stdout);
        fflush(stdout);
    }

    exit(0); /* control never reaches here */
}

/*
 * eval - Evaluate the command line that the user has just typed in
 *
 * If the user has requested a built-in command (quit, jobs, bg or fg)
 * then execute it immediately. Otherwise, fork a child process and
 * run the job in the context of the child. If the job is running in
 * the foreground, wait for it to terminate and then return.  Note:
 * each child process must have a unique process group ID so that our
 * background children don't receive SIGINT (SIGTSTP) from the kernel
 * when we type ctrl-c (ctrl-z) at the keyboard.
 */
void eval(const char *const cmdline)
{
    int bg; /* should the job run in bg or fg? */
    struct cmdline_tokens tok;

    /* Parse command line */
    bg = parseline(cmdline, &tok);

    if (bg == -1) /* parsing error */
        return;
    if (tok.argv[0] == NULL) /* ignore empty lines */
        return;

    sigset_t sig_chld_mask, old_sig_set;

    switch (tok.builtins)
    {
    case BUILTIN_BG:
        builtin_kill_bg_fg_trait(tok.argv[1], BUILTIN_BG);
        break;
    case BUILTIN_FG:
        builtin_kill_bg_fg_trait(tok.argv[1], BUILTIN_FG);
        break;
    case BUILTIN_JOBS:
        builtin_jobs(tok.outfile);
        break;
    case BUILTIN_KILL:
        builtin_kill_bg_fg_trait(tok.argv[1], BUILTIN_KILL);
        break;
    case BUILTIN_NONE:
        run_outside_command(tok.argv, tok.infile, tok.outfile, cmdline, bg);
        break;
    case BUILTIN_NOHUP:
        handle_system_call(sigemptyset(&sig_chld_mask));
        handle_system_call(sigaddset(&sig_chld_mask, SIGHUP));
        handle_system_call(
            sigprocmask(SIG_BLOCK, &sig_chld_mask, &old_sig_set));
        run_outside_command(tok.argv + 1, tok.infile, tok.outfile, cmdline, bg);
        handle_system_call(sigprocmask(SIG_SETMASK, &sig_chld_mask, NULL));
        break;
    case BUILTIN_QUIT:
        exit(0);
    }

    return;
}

/**
 * `run_outside_command` 执行外部命令.

 * 参数:
 *
 *  - `argv`: 切割过的命令行
 *  - `in_file`: 重定向的输入文件路径
 *  - `out_file`: 重定向的输出文件路径
 *  - `is_bg`: 标志是否是后台任务
 */
void run_outside_command(char *const *argv, const char *const in_file,
                         const char *const out_file, const char *const cmd_line,
                         const int is_bg)
{
    sigset_t mask, old_sig_set;
    handle_system_call(sigfillset(&mask));
    handle_system_call(sigprocmask(SIG_BLOCK, &mask, &old_sig_set));

    // 在这里检查是否可以创建新的任务
    int i;
    for (i = 0; i < MAXJOBS; i++)
    {
        if (job_list[i].pid == 0)
            break;
    }
    // 如果任务数量已经达到上限, 退出函数.
    if (i == MAXJOBS)
    {
        puts("Tried to create too many jobs.");
        fflush(stdout);
        handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));
        return;
    }

    pid_t pid = 0;
    handle_system_call(pid = fork());
    if (pid == 0)
    {
        handle_system_call(setpgid(0, 0));
        handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));
        int in_file_description = -1, out_file_description = -1;

        // 处理重定向.
        if (in_file != NULL)
        {
            handle_system_call(in_file_description = open(in_file, O_RDONLY));
            handle_system_call(dup2(in_file_description, STDIN_FILENO));
            handle_system_call(close(in_file_description));
        }

        if (out_file != NULL)
        {
            handle_system_call(out_file_description =
                                   open(out_file, O_WRONLY | O_TRUNC | O_CREAT,
                                        S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH));
            handle_system_call(dup2(out_file_description, STDOUT_FILENO));
            handle_system_call(close(out_file_description));
        }

        if (execve(argv[0], argv, environ) == -1)
        {
            handle_system_call(sio_puts(argv[0]));
            handle_system_call(sio_puts(": Command not found\n"));
            exit(0);
        }
    }
    else
    {
        if (is_bg)
        {
            addjob(job_list, pid, BG, cmd_line);
            int jid = nextjid;
            if (jid == 0)
                jid = MAXJID;
            else
                jid--;

            handle_system_call(sio_put("[%d] (%d) ", jid, pid));
            handle_system_call(sio_puts(cmd_line));
            handle_system_call(sio_puts("\n"));
            handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));
        }
        else
        {
            handle_system_call(addjob(job_list, pid, FG, cmd_line));
            handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));
            wait_for_fg_exit();
        }
    }
}

/**
 * `wait_for_fg_exit`: 等待前台任务退出.
 */
void wait_for_fg_exit()
{
    sigset_t empty_mask, fill_mask, old_sig_set;
    handle_system_call(sigemptyset(&empty_mask));
    handle_system_call(sigfillset(&fill_mask));
    handle_system_call(sigprocmask(SIG_BLOCK, &fill_mask, &old_sig_set));

    while (fgpid(job_list) > 0)
        sigsuspend(&empty_mask);
    handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));
}

/**
 * `builtin_jobs` 执行内置命令 `jobs`
 *
 * 参数: `out_file`: 输出文件的路径. 为 `NULL` 时, 输出到终端.
 */
void builtin_jobs(const char *const out_file)
{
    int out_file_description = STDOUT_FILENO;
    if (out_file != NULL)
    {
        out_file_description = open(out_file, O_WRONLY | O_TRUNC | O_CREAT,
                                    S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

        if (out_file_description == -1)
        {
            printf("Error: %s\n", strerror(errno));
            fflush(stdout);
            return;
        }
    }

    sigset_t mask, old_sig_set;
    handle_system_call(sigfillset(&mask));
    handle_system_call(sigprocmask(SIG_BLOCK, &mask, &old_sig_set));

    listjobs(job_list, out_file_description);

    handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));
    if (out_file_description != STDOUT_FILENO)
    {
        handle_system_call(close(out_file_description));
    }
}

/**
 * `builtin_kill_bg_fg_trait`: 为内置的 `kill` `bg` `fg` 命令提供统一的接口.
 * 解析其后的`pid`或`%jid`, 在不合法时发出警告, 合法时执行命令.
 *
 * 参数:
 *
 * - `arg`: 用户输入的命令参数.
 *
 * - `command_type`: 命令的类型, 可以是 `BUILTIN_KILL_TYPE` `BUILTIN_FG_TYPE`
 * 或者 `BUILTIN_BG_TYPE`. 调用者需要保证 `command_type` 一定是三者之一.
 */
void builtin_kill_bg_fg_trait(const char *const arg,
                              const enum builtins_t command_type)
{
    sigset_t mask, old_sig_set;
    handle_system_call(sigfillset(&mask));
    handle_system_call(sigprocmask(SIG_BLOCK, &mask, &old_sig_set));
    int pid = 0, jid = 0;
    if (arg == NULL)
    {
        printf("%s command requires PID or %%jobid argument\n",
               command_name[command_type]);
        fflush(stdout);
        goto end;
    }

    if (arg[0] == '%')
    {
        if (sscanf(arg + 1, "%d", &jid) != 1)
        {
            printf("%s: argument must be a PID or %%jobid\n",
                   command_name[command_type]);
            fflush(stdout);
            goto end;
        }

        if (jid >= 0)
        {
            pid = is_valid_jid(jid);
            if (!pid)
            {
                printf("%%%d: No such job\n", jid);
                fflush(stdout);
                goto end;
            }
            goto impl;
        }

        else if (jid < 0)
        {
            pid = is_valid_jid(-jid);
            if (!pid)
            {
                printf("%%%d: No such process group\n", jid);
                fflush(stdout);
                goto end;
            }
            goto impl;
        }
    }
    else
    {
        if (sscanf(arg, "%d", &pid) != 1)
        {
            printf("%s: argument must be a PID or %%jobid\n",
                   command_name[command_type]);
            fflush(stdout);
            goto end;
        }
        if (pid >= 0)
        {
            int valid = is_valid_jid(pid);
            if (!valid)
            {
                printf("(%d): No such process\n", pid);
                fflush(stdout);
                goto end;
            }
            goto impl;
        }

        else if (pid < 0)
        {
            int valid = is_valid_jid(-pid);
            if (!valid)
            {
                printf("(%d): No such process group\n", -pid);
                fflush(stdout);
                goto end;
            }
            goto impl;
        }
    }

impl:
    builtin_kill_bg_fg_impl(command_type, pid);
    handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));
    if (command_type == BUILTIN_FG)
        wait_for_fg_exit();
    return;

end:
    handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));
    return;
}

/**
 * `builtin_kill_bg_fg_impl` `kill` `bg` `fg` 的核心实现.
 *
 * 参数:
 *
 *  - `command_type` : 命令的类型, 可以是 `BUILTIN_KILL_TYPE` `BUILTIN_FG_TYPE`
 * 或者 `BUILTIN_BG_TYPE`. 调用者需要保证 `command_type` 一定是三者之一.
 *
 *  - `pid`: 需要发送信号的 pid. 调用者需要保证其合法.
 *
 * 注意: `builtin_kill_bg_fg_impl` 涉及到对 `job_list` 的访问,
 * 调用前需要注意信号阻塞.
 */
void builtin_kill_bg_fg_impl(const enum builtins_t command_type, const int pid)
{
    struct job_t *job_ptr;
    switch (command_type)
    {
    case BUILTIN_KILL:
        handle_system_call(kill(-pid, SIGTERM));
        break;
    case BUILTIN_FG:
        job_ptr = getjobpid(job_list, pid);
        job_ptr->state = FG;
        handle_system_call(kill(-pid, SIGCONT));
        break;
    case BUILTIN_BG:
        job_ptr = getjobpid(job_list, pid);
        printf("[%d] (%d) %s\n", job_ptr->jid, pid, job_ptr->cmdline);
        fflush(stdout);
        job_ptr->state = BG;
        handle_system_call(kill(-pid, SIGCONT));
        break;
    default: // 我不好评价.
             // 一方面, 控制流永远不会到达 default, 故而是 dead code, 要扣分;
             // 另一方面, 不加 default, 编译会 error.
        break;
    }
}

/**
 * `is_valid_pid` 判断一个 `pid` 是否合法.
 * 参数: 一个 `pid`, `int` 类型.
 * 返回值: 如果 `pid` 合法, 返回其自身. 如果不合法，返回 `0`.
 *
 * 注意: `is_valid_pid` 涉及到对 `job_list` 的访问, 调用前需要注意信号阻塞.
 */
int is_valid_pid(const int pid)
{
    if (pid <= 0)
        return 0;
    const struct job_t *const job_ptr = getjobpid(job_list, pid);
    return job_ptr == NULL ? 0 : pid;
}

/**
 * `is_valid_jid` 判断一个 `jid` 是否合法.
 * 参数: 一个 `jid`, `int` 类型.
 * 返回值: 如果 `jid` 合法, 返回其对应的 `pid`. 如果不合法，返回 `0`.
 *
 * 注意: `is_valid_jid` 涉及到对 `job_list` 的访问, 调用前需要注意信号阻塞.
 */
int is_valid_jid(const int jid)
{
    if (jid <= 0)
        return 0;
    const struct job_t *const job_ptr = getjobjid(job_list, jid);
    return job_ptr == NULL ? 0 : job_ptr->pid;
}

/*
 * parseline - Parse the command line and build the argv array.
 *
 * Parameters:
 *   cmdline:  The command line, in the form:
 *
 *                command [arguments...] [< infile] [> oufile] [&]
 *
 *   tok:      Pointer to a cmdline_tokens structure. The elements of this
 *             structure will be populated with the parsed tokens. Characters
 *             enclosed in single or double quotes are treated as a single
 *             argument.
 * Returns:
 *   1:        if the user has requested a BG job
 *   0:        if the user has requested a FG job
 *  -1:        if cmdline is incorrectly formatted
 *
 * Note:       The string elements of tok (e.g., argv[], infile, outfile)
 *             are statically allocated inside parseline() and will be
 *             overwritten the next time this function is invoked.
 */
int parseline(const char *cmdline, struct cmdline_tokens *tok)
{

    static char array[MAXLINE];        /* holds local copy of command line */
    const char delims[10] = " \t\r\n"; /* argument delimiters (white-space) */
    char *buf = array;                 /* ptr that traverses command line */
    char *next;                        /* ptr to the end of the current arg */
    char *endbuf;                      /* ptr to end of cmdline string */
    int is_bg;                         /* background job? */

    int parsing_state; /* indicates if the next token is the
                          input or output file */

    if (cmdline == NULL)
    {
        (void)fprintf(stderr, "Error: command line is NULL\n");
        return -1;
    }

    (void)strncpy(buf, cmdline, MAXLINE);
    endbuf = buf + strlen(buf);

    tok->infile = NULL;
    tok->outfile = NULL;

    /* Build the argv list */
    parsing_state = ST_NORMAL;
    tok->argc = 0;

    while (buf < endbuf)
    {
        /* Skip the white-spaces */
        buf += strspn(buf, delims);
        if (buf >= endbuf)
            break;

        /* Check for I/O redirection specifiers */
        if (*buf == '<')
        {
            if (tok->infile)
            {
                (void)fprintf(stderr, "Error: Ambiguous I/O redirection\n");
                return -1;
            }
            parsing_state |= ST_INFILE;
            buf++;
            continue;
        }
        if (*buf == '>')
        {
            if (tok->outfile)
            {
                (void)fprintf(stderr, "Error: Ambiguous I/O redirection\n");
                return -1;
            }
            parsing_state |= ST_OUTFILE;
            buf++;
            continue;
        }

        if (*buf == '\'' || *buf == '\"')
        {
            /* Detect quoted tokens */
            buf++;
            next = strchr(buf, *(buf - 1));
        }
        else
        {
            /* Find next delimiter */
            next = buf + strcspn(buf, delims);
        }

        if (next == NULL)
        {
            /* Returned by strchr(); this means that the closing
               quote was not found. */
            (void)fprintf(stderr, "Error: unmatched %c.\n", *(buf - 1));
            return -1;
        }

        /* Terminate the token */
        *next = '\0';

        /* Record the token as either the next argument or the i/o file */
        switch (parsing_state)
        {
        case ST_NORMAL:
            tok->argv[tok->argc++] = buf;
            break;
        case ST_INFILE:
            tok->infile = buf;
            break;
        case ST_OUTFILE:
            tok->outfile = buf;
            break;
        default:
            (void)fprintf(stderr, "Error: Ambiguous I/O redirection\n");
            return -1;
        }
        parsing_state = ST_NORMAL;

        /* Check if argv is full */
        if (tok->argc >= MAXARGS - 1)
            break;

        buf = next + 1;
    }

    if (parsing_state != ST_NORMAL)
    {
        (void)fprintf(stderr,
                      "Error: must provide file name for redirection\n");
        return -1;
    }

    /* The argument list must end with a NULL pointer */
    tok->argv[tok->argc] = NULL;

    if (tok->argc == 0) /* ignore blank line */
        return 1;

    if (!strcmp(tok->argv[0], "quit"))
    { /* quit command */
        tok->builtins = BUILTIN_QUIT;
    }
    else if (!strcmp(tok->argv[0], "jobs"))
    { /* jobs command */
        tok->builtins = BUILTIN_JOBS;
    }
    else if (!strcmp(tok->argv[0], "bg"))
    { /* bg command */
        tok->builtins = BUILTIN_BG;
    }
    else if (!strcmp(tok->argv[0], "fg"))
    { /* fg command */
        tok->builtins = BUILTIN_FG;
    }
    else if (!strcmp(tok->argv[0], "kill"))
    { /* kill command */
        tok->builtins = BUILTIN_KILL;
    }
    else if (!strcmp(tok->argv[0], "nohup"))
    { /* kill command */
        tok->builtins = BUILTIN_NOHUP;
    }
    else
    {
        tok->builtins = BUILTIN_NONE;
    }

    /* Should the job run in the background? */
    if ((is_bg = (*tok->argv[tok->argc - 1] == '&')) != 0)
        tok->argv[--tok->argc] = NULL;

    return is_bg;
}

/*****************
 * Signal handlers
 *****************/

/*
 * sigchld_handler - The kernel sends a SIGCHLD to the shell whenever
 *     a child job terminates (becomes a zombie), or stops because it
 *     received a SIGSTOP, SIGTSTP, SIGTTIN or SIGTTOU signal. The
 *     handler reaps all available zombie children, but doesn't wait
 *     for any other currently running children to terminate.
 */
void sigchld_handler(int sig)
{
    const int old_errno = errno;
    int status;

    pid_t pid;

    sigset_t mask, old_sig_set;
    handle_system_call(sigfillset(&mask));
    handle_system_call(sigprocmask(SIG_BLOCK, &mask, &old_sig_set));

    while ((pid = waitpid(-1, &status, WNOHANG | WUNTRACED | WCONTINUED)) > 0)
    {
        if (WIFSTOPPED(status))
        {
            struct job_t *const job_ptr = getjobpid(job_list, pid);
            if (job_ptr == NULL)
                app_error(VAL_TO_STR(__LINE__));
            handle_system_call(sio_put("Job [%d] (%d) stopped by signal %d\n",
                                       job_ptr->jid, pid, WSTOPSIG(status)));
            job_ptr->state = ST;
        }
        else if (WIFSIGNALED(status))
        {
            const struct job_t *const job_ptr = getjobpid(job_list, pid);
            if (job_ptr == NULL)
                app_error(VAL_TO_STR(__LINE__));
            handle_system_call(
                sio_put("Job [%d] (%d) terminated by signal %d\n", job_ptr->jid,
                        pid, WTERMSIG(status)));
            if (deletejob(job_list, pid) == 0)
                app_error(VAL_TO_STR(__LINE__));
        }
        else if (WIFEXITED(status))
        {
            if (deletejob(job_list, pid) == 0)
                app_error(VAL_TO_STR(__LINE__));
        }
        else if (WIFCONTINUED(status))
        {
            struct job_t *const job_ptr = getjobpid(job_list, pid);
            if (job_ptr == NULL)
                app_error(VAL_TO_STR(__LINE__));
            job_ptr->state = BG;
        }
    }

    handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));
    errno = old_errno;
}

/*
 * sigint_handler - The kernel sends a SIGINT to the shell whenver the
 *    user types ctrl-c at the keyboard.  Catch it and send it along
 *    to the foreground job.
 */
void sigint_handler(int sig)
{
    const int old_errno = errno;

    sigset_t mask, old_sig_set;
    handle_system_call(sigfillset(&mask));
    handle_system_call(sigprocmask(SIG_BLOCK, &mask, &old_sig_set));

    const pid_t fg_pid = fgpid(job_list);
    if (fg_pid > 0)
    {
        handle_system_call(kill(-fg_pid, SIGINT));
    }

    handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));

    errno = old_errno;
}

/*
 * sigtstp_handler - The kernel sends a SIGTSTP to the shell whenever
 *     the user types ctrl-z at the keyboard. Catch it and suspend the
 *     foreground job by sending it a SIGTSTP.
 */
void sigtstp_handler(int sig)
{
    const int old_errno = errno;

    sigset_t mask, old_sig_set;
    handle_system_call(sigfillset(&mask));
    handle_system_call(sigprocmask(SIG_BLOCK, &mask, &old_sig_set));

    const pid_t fg_pid = fgpid(job_list);

    if (fg_pid > 0)
    {
        handle_system_call(kill(-fg_pid, SIGTSTP));
    }

    handle_system_call(sigprocmask(SIG_SETMASK, &old_sig_set, NULL));

    errno = old_errno;
}

/*
 * sigquit_handler - The driver program can gracefully terminate the
 *    child shell by sending it a SIGQUIT signal.
 */
void sigquit_handler(int sig)
{
    sio_error("Terminating after receipt of SIGQUIT signal\n");
}

/*********************
 * End signal handlers
 *********************/

/***********************************************
 * Helper routines that manipulate the job list
 **********************************************/

/* clearjob - Clear the entries in a job struct */
void clearjob(struct job_t *job)
{
    job->pid = 0;
    job->jid = 0;
    job->state = UNDEF;
    job->cmdline[0] = '\0';
}

/* initjobs - Initialize the job list */
void initjobs(struct job_t *job_list)
{
    int i;

    for (i = 0; i < MAXJOBS; i++)
        clearjob(&job_list[i]);
}

/* maxjid - Returns largest allocated job ID */
int maxjid(struct job_t *job_list)
{
    int i, max = 0;

    for (i = 0; i < MAXJOBS; i++)
        if (job_list[i].jid > max)
            max = job_list[i].jid;
    return max;
}

/* addjob - Add a job to the job list */
int addjob(struct job_t *job_list, pid_t pid, int state, const char *cmdline)
{
    int i;

    if (pid < 1)
        return 0;

    for (i = 0; i < MAXJOBS; i++)
    {
        if (job_list[i].pid == 0)
        {
            job_list[i].pid = pid;
            job_list[i].state = state;
            job_list[i].jid = nextjid++;
            if (nextjid > MAXJOBS)
                nextjid = 1;
            strcpy(job_list[i].cmdline, cmdline);
            if (verbose)
            {
                printf("Added job [%d] %d %s\n", job_list[i].jid,
                       job_list[i].pid, job_list[i].cmdline);
            }
            return 1;
        }
    }
    printf("Tried to create too many jobs\n");
    return 0;
}

/* deletejob - Delete a job whose PID=pid from the job list */
int deletejob(struct job_t *job_list, pid_t pid)
{
    int i;

    if (pid < 1)
        return 0;

    for (i = 0; i < MAXJOBS; i++)
    {
        if (job_list[i].pid == pid)
        {
            clearjob(&job_list[i]);
            nextjid = maxjid(job_list) + 1;
            return 1;
        }
    }
    return 0;
}

/* fgpid - Return PID of current foreground job, 0 if no such job */
pid_t fgpid(struct job_t *job_list)
{
    int i;

    for (i = 0; i < MAXJOBS; i++)
        if (job_list[i].state == FG)
            return job_list[i].pid;
    return 0;
}

/* getjobpid  - Find a job (by PID) on the job list */
struct job_t *getjobpid(struct job_t *job_list, pid_t pid)
{
    int i;

    if (pid < 1)
        return NULL;
    for (i = 0; i < MAXJOBS; i++)
        if (job_list[i].pid == pid)
            return &job_list[i];
    return NULL;
}

/* getjobjid  - Find a job (by JID) on the job list */
struct job_t *getjobjid(struct job_t *job_list, int jid)
{
    int i;

    if (jid < 1)
        return NULL;
    for (i = 0; i < MAXJOBS; i++)
        if (job_list[i].jid == jid)
            return &job_list[i];
    return NULL;
}

/* pid2jid - Map process ID to job ID */
int pid2jid(pid_t pid)
{
    int i;

    if (pid < 1)
        return 0;
    for (i = 0; i < MAXJOBS; i++)
        if (job_list[i].pid == pid)
        {
            return job_list[i].jid;
        }
    return 0;
}

/* listjobs - Print the job list */
void listjobs(struct job_t *job_list, int output_fd)
{
    int i;
    char buf[MAXLINE << 2];

    for (i = 0; i < MAXJOBS; i++)
    {
        memset(buf, '\0', MAXLINE);
        if (job_list[i].pid != 0)
        {
            sprintf(buf, "[%d] (%d) ", job_list[i].jid, job_list[i].pid);
            if (write(output_fd, buf, strlen(buf)) < 0)
            {
                fprintf(stderr, "Error writing to output file\n");
                exit(1);
            }
            memset(buf, '\0', MAXLINE);
            switch (job_list[i].state)
            {
            case BG:
                sprintf(buf, "Running    ");
                break;
            case FG:
                sprintf(buf, "Foreground ");
                break;
            case ST:
                sprintf(buf, "Stopped    ");
                break;
            default:
                sprintf(buf, "listjobs: Internal error: job[%d].state=%d ", i,
                        job_list[i].state);
            }
            if (write(output_fd, buf, strlen(buf)) < 0)
            {
                fprintf(stderr, "Error writing to output file\n");
                exit(1);
            }
            memset(buf, '\0', MAXLINE);
            sprintf(buf, "%s\n", job_list[i].cmdline);
            if (write(output_fd, buf, strlen(buf)) < 0)
            {
                fprintf(stderr, "Error writing to output file\n");
                exit(1);
            }
        }
    }
}
/******************************
 * end job list helper routines
 ******************************/

/***********************
 * Other helper routines
 ***********************/

/*
 * usage - print a help message
 */
void usage(void)
{
    printf("Usage: shell [-hvp]\n");
    printf("   -h   print this message\n");
    printf("   -v   print additional diagnostic information\n");
    printf("   -p   do not emit a command prompt\n");
    exit(1);
}

/*
 * unix_error - unix-style error routine
 */
void unix_error(char *msg)
{
    fprintf(stdout, "%s: %s\n", msg, strerror(errno));
    exit(1);
}

/*
 * app_error - application-style error routine
 */
void app_error(char *msg)
{
    fprintf(stdout, "%s\n", msg);
    exit(1);
}

/* Private sio_functions */
/* sio_reverse - Reverse a string (from K&R) */
static void sio_reverse(char s[])
{
    int c, i, j;

    for (i = 0, j = strlen(s) - 1; i < j; i++, j--)
    {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

/* sio_ltoa - Convert long to base b string (from K&R) */
static void sio_ltoa(long v, char s[], int b)
{
    int c, i = 0;

    do
    {
        s[i++] = ((c = (v % b)) < 10) ? c + '0' : c - 10 + 'a';
    } while ((v /= b) > 0);
    s[i] = '\0';
    sio_reverse(s);
}

/* sio_strlen - Return length of string (from K&R) */
static size_t sio_strlen(const char s[])
{
    int i = 0;

    while (s[i] != '\0')
        ++i;
    return i;
}

/* sio_copy - Copy len chars from fmt to s (by Ding Rui) */
void sio_copy(char *s, const char *fmt, size_t len)
{
    if (!len)
        return;

    for (size_t i = 0; i < len; i++)
        s[i] = fmt[i];
}

/* Public Sio functions */
ssize_t sio_puts(const char s[]) /* Put string */
{
    return write(STDOUT_FILENO, s, sio_strlen(s));
}

ssize_t sio_putl(long v) /* Put long */
{
    char s[128];

    sio_ltoa(v, s, 10); /* Based on K&R itoa() */
    return sio_puts(s);
}

ssize_t sio_put(const char *fmt,
                ...) // Put to the console. only understands %d
{
    va_list ap;
    char str[MAXLINE]; // formatted string
    char arg[128];
    const char *mess = "sio_put: Line too long!\n";
    int i = 0, j = 0;
    int sp = 0;
    int v;

    if (fmt == 0)
        return -1;

    va_start(ap, fmt);
    while (fmt[j])
    {
        if (fmt[j] != '%')
        {
            j++;
            continue;
        }

        sio_copy(str + sp, fmt + i, j - i);
        sp += j - i;

        switch (fmt[j + 1])
        {
        case 0:
            va_end(ap);
            if (sp >= MAXLINE)
            {
                write(STDOUT_FILENO, mess, sio_strlen(mess));
                return -1;
            }

            str[sp] = 0;
            return write(STDOUT_FILENO, str, sp);

        case 'd':
            v = va_arg(ap, int);
            sio_ltoa(v, arg, 10);
            sio_copy(str + sp, arg, sio_strlen(arg));
            sp += sio_strlen(arg);
            i = j + 2;
            j = i;
            break;

        case '%':
            sio_copy(str + sp, "%", 1);
            sp += 1;
            i = j + 2;
            j = i;
            break;

        default:
            sio_copy(str + sp, fmt + j, 2);
            sp += 2;
            i = j + 2;
            j = i;
            break;
        }
    } // end while

    sio_copy(str + sp, fmt + i, j - i);
    sp += j - i;

    va_end(ap);
    if (sp >= MAXLINE)
    {
        write(STDOUT_FILENO, mess, sio_strlen(mess));
        return -1;
    }

    str[sp] = 0;
    return write(STDOUT_FILENO, str, sp);
}

void sio_error(char s[]) /* Put error message and exit */
{
    sio_puts(s);
    _exit(1);
}

/*
 * Signal - wrapper for the sigaction function
 */
handler_t *Signal(int signum, handler_t *handler)
{
    struct sigaction action, old_action;

    action.sa_handler = handler;
    sigemptyset(&action.sa_mask); /* block sigs of type being handled */
    action.sa_flags = SA_RESTART; /* restart syscalls if possible */

    if (sigaction(signum, &action, &old_action) < 0)
        unix_error("Signal error");
    return (old_action.sa_handler);
}
