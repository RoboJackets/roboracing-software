#include <ros/ros.h>
#include <ros/publisher.h>

#define STATE_LIGHT_WAITING 1
#define STATE_DRAGRACE_GO 2
#define STATE_DRAGRACE_STOP 4
#define STATE_VALID 7

char global_state;
void transition_state();
char get_state();
int main(int argc, char** argv)
{
	global_state = STATE_LIGHT_WAITING;
	while((global_state & STATE_VALID)) {
		if(global_state == STATE_LIGHT_WAITING && true) { //light turns green
			transition_state();
		}

		if(global_state == STATE_DRAGRACE_GO && true) { //wall sensed
			transition_state();
		}

		if(global_state == STATE_DRAGRACE_STOP && true) { //timer expired
			transition_state();
		}
	}
	std::exit(1);
}

void transition_state()
{
	global_state << 1;
}

char get_state()
{
	return global_state;
}


