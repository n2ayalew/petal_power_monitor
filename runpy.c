#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main()
{
	setuid( 0 );
	system( "./run_disaggregation.py" );

	return 0;
}
