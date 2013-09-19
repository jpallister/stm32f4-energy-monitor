#include "helper.h"
#include <cstring>
#include <iostream>
#include <sys/types.h>
#include <signal.h>
#include <readline/readline.h>

using namespace std;

void mt_start_output()
{
    int i;

    cout << "\r";
    for(i = 0; i < strlen(rl_line_buffer) + strlen(rl_prompt); ++i)
        cout << " ";
    cout << "\r";
}

void mt_end_output()
{
    raise(SIGWINCH);
}
